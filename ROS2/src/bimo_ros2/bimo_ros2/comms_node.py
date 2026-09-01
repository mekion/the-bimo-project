"""
Copyright (c) 2026, Mekion
SPDX-License-Identifier: Apache-2.0

ROS2 node wrapping the Bimo API
"""

import itertools
import queue
import threading
from concurrent.futures import Future

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from bimo_ros2.bimo_core import Bimo
from bimo_ros2_msgs.msg import ActionCommand, StateData
from bimo_ros2_msgs.srv import AddRoutine, PerformRoutine, GetRoutine, CaptureImage


class BimoCommsNode(Node):
    # Comms priorities (lower = serviced first by the worker thread)
    PRIO_ACTION = 0
    PRIO_ALIVE = 1
    PRIO_HEADING = 1
    PRIO_ROUTINE = 2
    PRIO_CALIBRATE = 2
    PRIO_STATE = 3

    def __init__(self):
        super().__init__('bimo_comms')

        self.declare_parameter('state_rate_hz', 20.0)
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('timeout', 0.2)
        self.declare_parameter('calibrate', False)
        self.declare_parameter('camera_resolution', [1280, 720])

        rate = self.get_parameter('state_rate_hz').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        calibrate = self.get_parameter('calibrate').value
        cam_res = tuple(self.get_parameter('camera_resolution').value)

        self._counter = itertools.count()
        self._jobs = queue.PriorityQueue()
        self.busy = threading.Event()
        self._stop = threading.Event()
        self._state_poll_pending = threading.Event()
        self._bridge = CvBridge()
        self._cam_locks = {'front': threading.Lock(), 'top': threading.Lock()}

        self.bimo = Bimo()
        self.get_logger().info('Initializing Bimo hardware...')
        self.bimo.initialize(calibrate=calibrate, baudrate=baudrate,
                              timeout=timeout, camera_resolution=cam_res)
        self.get_logger().info('Bimo ready.')

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        # Fast callbacks (short, non-blocking beyond the worker's own execution time)
        # share one group so they're still serialized relative to each other,
        # but this group is separate from the long-blocking one below.
        fast_group = MutuallyExclusiveCallbackGroup()

        # Long blocking services (perform_routine, calibrate) get a group of their
        # own so they never prevent fast_group callbacks (like on_action) from
        # running while they're mid-wait on fut.result().
        blocking_group = ReentrantCallbackGroup()

        self.action_sub = self.create_subscription(
            ActionCommand, 'bimo/cmd_action', self.on_action, 10,
            callback_group=fast_group)

        self.state_pub = self.create_publisher(StateData, 'bimo/state', 10)

        self.alive_srv = self.create_service(
            Trigger, 'bimo/alive', self.on_alive,
            callback_group=fast_group)

        self.add_routine_srv = self.create_service(
            AddRoutine, 'bimo/add_routine', self.on_add_routine,
            callback_group=fast_group)

        self.get_routine_srv = self.create_service(
            GetRoutine, 'bimo/get_routine', self.on_get_routine,
            callback_group=fast_group)

        self.lock_heading_srv = self.create_service(
            Trigger, 'bimo/lock_heading', self.on_lock_heading,
            callback_group=fast_group)

        self.unlock_heading_srv = self.create_service(
            Trigger, 'bimo/unlock_heading', self.on_unlock_heading,
            callback_group=fast_group)

        self.capture_image_srv = self.create_service(
            CaptureImage, 'bimo/capture_image', self.on_capture_image,
            callback_group=fast_group)

        self.state_timer = self.create_timer(
            1.0 / rate, self.on_state_timer, callback_group=fast_group)

        # Long blocking services
        self.routine_srv = self.create_service(
            PerformRoutine, 'bimo/perform_routine', self.on_perform_routine,
            callback_group=blocking_group)

        self.calibrate_srv = self.create_service(
            Trigger, 'bimo/calibrate', self.on_calibrate,
            callback_group=blocking_group)

    # ----- WORKER -----

    def _submit(self, priority, func):
        fut = Future()
        seq = next(self._counter)

        def _run():
            try:
                fut.set_result(func())

            except Exception as exc:
                fut.set_exception(exc)

        self._jobs.put((priority, seq, _run))
        return fut

    def _worker_loop(self):
        while not self._stop.is_set():
            try:
                _, _, job = self._jobs.get(timeout=0.5)

            except queue.Empty:
                continue

            job()

    def destroy_node(self):
        self._stop.set()
        self._worker.join(timeout=2.0)

        try:
            if self.bimo.mcu is not None:
                self.bimo.mcu.close()

        except Exception as exc:
            self.get_logger().warn(f'Error closing serial port: {exc}')

        for cam_attr in ('front_cam', 'top_cam'):
            cam = getattr(self.bimo, cam_attr, None)
            if cam is not None:
                try:
                    cam.release()
                except Exception as exc:
                    self.get_logger().warn(f'Error releasing {cam_attr}: {exc}')

        super().destroy_node()

    # ----- STATE / ACTION -----

    def on_state_timer(self):
        if self._state_poll_pending.is_set():
            return  # previous poll still in flight, skip this tick rather than pile up

        self._state_poll_pending.set()
        fut = self._submit(self.PRIO_STATE, self._do_state_poll)
        fut.add_done_callback(self._on_state_poll_done)

    def _on_state_poll_done(self, fut):
        self._state_poll_pending.clear()
        exc = fut.exception()

        if exc is not None:
            self.get_logger().warn(f'State poll failed: {exc}')

    def _do_state_poll(self):
        state = self.bimo.request_state_data()
        msg = StateData()
        msg.orient = state['orient']
        msg.distances = state['distances']
        msg.power = state['power']
        msg.rp_temp = state['rp_temp']
        msg.servo_pos = state['servo_pos']
        msg.servo_speed = state['servo_speed']
        msg.servo_load = state['servo_load']
        msg.servo_voltage = state['servo_voltage']
        msg.servo_current = state['servo_current']
        msg.servo_temp = list(state['servo_temp'])
        self.state_pub.publish(msg)

    def on_action(self, msg):
        if self.busy.is_set():
            self.get_logger().warn('Dropping cmd_action: routine/calibration in progress')
            return

        self._submit(self.PRIO_ACTION,
                     lambda: self.bimo.send_positions(list(msg.positions)))

    # ----- SERVICES -----

    def on_alive(self, request, response):
        fut = self._submit(self.PRIO_ALIVE, self.bimo.available)
        try:
            response.success = fut.result(timeout=2.0)
            response.message = 'alive' if response.success else 'no response from MCU'

        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response

    def on_perform_routine(self, request, response):
        self.busy.set()
        fut = self._submit(self.PRIO_ROUTINE, lambda: self.bimo.perform(request.name))

        try:
            fut.result(timeout=15.0)
            response.success = True
            response.message = f"routine '{request.name}' completed"

        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(str(exc))

        finally:
            self.busy.clear()

        return response

    def on_add_routine(self, request, response):
        poses = []

        for step in request.steps:
            pose = list(step.positions)
            poses.append(pose)

        def _register():
            self.bimo.routines.add_routine(request.name, poses)

        fut = self._submit(self.PRIO_ROUTINE, _register)

        try:
            fut.result(timeout=2.0)
            response.success = True
            response.message = f"routine '{request.name}' registered ({len(poses)} steps)"

        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response

    def on_get_routine(self, request, response):
        try:
            poses = self.bimo.routines.get_routine(request.name)
            response.steps = []

            for pose in poses:
                step = ActionCommand()
                step.positions = list(pose)
                response.steps.append(step)

            response.success = True
            response.message = f"routine '{request.name}' has {len(poses)} steps"

        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response

    def on_calibrate(self, request, response):
        self.busy.set()
        self.get_logger().info('Starting interactive calibration on this terminal...')
        fut = self._submit(self.PRIO_CALIBRATE, self.bimo.calibrate)

        try:
            fut.result(timeout=300.0)
            response.success = True
            response.message = 'calibration completed'

        except Exception as exc:
            response.success = False
            response.message = str(exc)

        finally:
            self.busy.clear()

        return response

    def on_lock_heading(self, request, response):
        fut = self._submit(self.PRIO_HEADING, self.bimo.lock_heading)
        try:
            fut.result(timeout=2.0)
            response.success = True
            response.message = 'heading locked'

        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response

    def on_unlock_heading(self, request, response):
        self.bimo.unlock_heading()
        response.success = True
        response.message = 'heading unlocked'
        return response

    def on_capture_image(self, request, response):
        camera = request.camera if request.camera in ('front', 'top') else 'front'
        lock = self._cam_locks[camera]

        try:
            with lock:
                frame = self.bimo.capture_image(camera)

            response.image = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            response.success = True
            response.message = f'captured from {camera}'

        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response


def main(args=None):
    rclpy.init(args=args)

    try:
        node = BimoCommsNode()
    except Exception as exc:
        print(f'Failed to start bimo_comms: {exc}')
        rclpy.shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
