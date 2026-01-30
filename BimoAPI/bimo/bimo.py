# Copyright (c) 2025-2026, Mekion
# SPDX-License-Identifier: Apache-2.0
"""Core Bimo Robotics Kit control class"""

from time import sleep
import serial
import struct
import math
import cv2


class Bimo():
    """Bimo Robotics Kit control class"""

    def __init__(self):
        # MCU comms
        self.mcu = None
        self.state_format = "<4f4H8h8h8h8H8h8B"
        self.state_size = struct.calcsize(self.state_format)

        # Cameras
        self.front_cam = None
        self.top_cam = None

        # Servos
        self.servo_max = [90, 90, 90, 12, 140, 140, 93, 93]
        self.servo_min = [-90, -90, -12, -90, 0, 0, -93, -93]
        self.sit_pose = [-45, -45, 0, 0, 140, 140, 93, 93]
        self.stand_pose = [-30, -30, 0, 0, 60, 60, 30, 30]
        self.centers = [1509, 2587, 2048, 2048, 456, 3640, 2048, 2048]

        # IMU calibration offsets
        self.x_orient = 0
        self.y_orient = 0

    # ===== UTILITY =====
    def initialize(self, calibrate=False, baudrate=921600, timeout=0.2,
                   camera_resolution=(1280, 720)):
        """
        Initialize robot from sitting position.

        Args:
            calibrate: run interactive servo calibration process.
            baudrate: MCU serial baudrate (default: 921600)
            timeout: Serial read timeout in seconds (default: 0.2)
            camera_resolution: Camera resolution tuple (default: (1280, 720))
                               Cameras run at fixed 30fps MJPEG.

        Supported camera resolutions:
            (1280, 720), (848, 480), (800, 600), (640, 480),
            (640, 360), (352, 288), (320, 240), (160, 120)
        """
        self.init_mcu_comms(baudrate, timeout)
        self.init_cameras(camera_resolution)

        # Calibrates if necessary
        if calibrate:
            self.calibrate()

        # Resets robot to sitting position
        print("INFO: Resetting to Sit position.")
        self.send_positions(self.sit_pose)
        sleep(3)  # Allows robot to settle
        self.update_imu_offsets()

        print("INFO: Initialization complete! Bimo is ready to roll (walk).")

    def calibrate(self):
        """Calibrates Servos (interactive process)"""
        print("INFO: Starting Bimo Calibration.")
        print("Place robot on flat ground and align legs and head using the provided guides.")
        input("Press Enter to continue...")

        self.calibrate_servos()

        # Calibrates ankles on centered position
        print("Lift robot from ground and remove the leg guide. Ankles will move and calibrate.")
        input("Press Enter to continue...")

        curr_pose = [2048 for _ in range(8)]
        ankle_diff = int(92.54 / 360 * 4095)

        curr_pose[6] += ankle_diff
        curr_pose[7] -= ankle_diff

        self.send_positions(self.servo2deg(curr_pose))
        sleep(3)
        self.calibrate_servos()

        print("Place robot on the ground in sitting pose.")
        input("Press Enter to continue...")
        print("INFO: Bimo Calibration Successful!")

    def update_imu_offsets(self):
        """Updates IMU offset"""
        print("INFO: Calculating IMU offsets.")
        x_total = 0
        y_total = 0

        for _ in range(10):
            x, y, _ = self.request_state_data()["orient"]
            x_total += x
            y_total += y
            sleep(0.05)

        self.x_orient = -x_total / 10.0
        self.y_orient = -y_total / 10.0

    # ===== MCU COMMS =====
    def init_mcu_comms(self, baudrate, timeout):
        # Tries to connect to MCU 5 times
        for attempt in range(5):
            for port in range(5):
                try:
                    self.mcu = serial.Serial(
                        f"/dev/ttyACM{port}",
                        baudrate=baudrate,
                        timeout=timeout,
                    )
                    return

                except Exception:
                    continue

            # Waits 100ms before trying again
            sleep(0.1)

        # Raises exception if no connection established
        raise RuntimeError("ERROR: Could not connect to MCU!")

    def request_state_data(self):
        """
            Returns robot state dict

            "orient" -> Euler angles
            "distances" -> [Front, Back, Right, Left] (m)
            "servo_pos" -> Degrees
            "servo_speed" -> Rad/s
            "servo_load" -> Nm
            "servo_voltage" -> V
            "servo_current" -> A
            "servo_temp" -> Celsius

        """
        # Asks for state data
        self.mcu.write(struct.pack("2i", *[4, 1]))

        # Processes message data
        data = self.mcu.read(self.state_size)
        unpacked = struct.unpack(self.state_format, data)

        return {
            "orient": self.quaternion_to_euler(unpacked[:4]),
            "distances": [d * 0.001 for d in unpacked[4:8]],
            "servo_pos": self.servo2deg(unpacked[8:16]),
            "servo_speed": [d * 0.088 * (math.pi / 180) for d in unpacked[16:24]],
            "servo_load": [d * 2.942 / 1000 for d in unpacked[24:32]],
            "servo_voltage": [d * 0.1 for d in unpacked[32:40]],
            "servo_current": [d * 0.0065 for d in unpacked[40:48]],
            "servo_temp": list(unpacked[48:56]),
        }

    def send_positions(self, actions):
        """Sends list of actions in degrees to MCU"""
        act = self.deg2servo(actions)
        self.mcu.write(struct.pack("9i", *[32] + act))

    def calibrate_servos(self):
        """Sends calibration command"""
        self.mcu.write(struct.pack("2i", *[4, 3]))

    def available(self):
        """Returns True if MCU ready."""
        self.mcu.write(struct.pack("2i", *[4, 2]))
        status = struct.unpack("i", self.mcu.read(4))

        if status == 1:
            return True

        else:
            return False

    def port(self):
        return self.mcu.port

    # ===== CAMERAS =====
    def init_cameras(self, resolution):
        """Load Bimo cameras."""
        resolutions = [
            (160, 120),
            (320, 240),
            (352, 288),
            (640, 360),
            (640, 480),
            (800, 600),
            (848, 480),
            (1280, 720),
        ]

        if resolution not in resolutions:
            raise ValueError(
                f"ERROR: Resolution {resolution} not supported. "
                f"Valid Resolutions: {resolutions}"
            )

        detected = []

        for idx in range(10):
            try:
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                if cap.isOpened():
                    ret, _ = cap.read()
                    if ret:
                        detected.append(idx)
                        print(f"INFO: Camera found! (/dev/video{idx})")
                        cap.release()

                        if len(detected) == 2:
                            break

            except Exception:
                pass

        if len(detected) < 2:
            raise RuntimeError(f"ERROR: Expected 2 cameras, found {len(detected)}!")

        self.front_cam = cv2.VideoCapture(detected[0], cv2.CAP_V4L2)
        self.top_cam = cv2.VideoCapture(detected[1], cv2.CAP_V4L2)

        for cam in [self.front_cam, self.top_cam]:
            cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
            cam.set(cv2.CAP_PROP_FPS, 30)  # Fixed 30FPS for MJPG

        print(f"INFO: Bimo Cameras Ready!")

    def capture_image(self, camera="front"):
        """Captures single frame. Returns (H, W, 3) BGR numpy array."""
        if camera not in ["front", "top"]:
            raise ValueError("ERROR: Available cameras are 'front' and 'top'.")

        if self.front_cam is None or self.top_cam is None:
            raise RuntimeError("ERROR: Cameras not loaded. Call initialize() first.")

        cam = self.front_cam if camera == "front" else self.top_cam
        ret, frame = cam.read()

        if not ret:
            raise RuntimeError(f"ERROR: Failed to grab frame from {camera.capitalize()} camera!")

        return frame

    # ===== HELPERS =====
    def scale_value(self, values, mins, maxs):
        """
        Scale one value or a list of values from [min, max] to [-1, 1].
        """
        def _scale_scalar(v, vmin, vmax):
            if vmax == vmin:
                return 0.0
            scaled = (v - vmin) / (vmax - vmin) * 2.0 - 1.0
            return max(-1.0, min(1.0, scaled))

        # Scalar case
        if not isinstance(values, (list, tuple)):
            return _scale_scalar(values, mins, maxs)

        # List case - convert scalar mins/maxs to lists if needed
        if not isinstance(mins, (list, tuple)):
            mins = [mins] * len(values)
        if not isinstance(maxs, (list, tuple)):
            maxs = [maxs] * len(values)

        if not (len(values) == len(mins) == len(maxs)):
            raise ValueError("ERROR: 'values', 'mins', and 'maxs' must have the same length.")

        return [
            _scale_scalar(v, vmin, vmax)
            for v, vmin, vmax in zip(values, mins, maxs)
        ]

    def clip_actions(self, array):
        """Clip actions inplace, in degrees"""
        for i in range(len(array)):
            if array[i] < self.servo_min[i]:
                array[i] = self.servo_min[i]

            elif array[i] > self.servo_max[i]:
                array[i] = self.servo_max[i]

    def quaternion_to_euler(self, quat):
        """
        Converts a quaternion to Euler angles (roll, pitch, yaw).
        Quaternion order: [w, x, y, z].
        Returns: [roll, pitch, yaw] in radians (X, Y, Z axes).
        """
        w, x, y, z = quat

        # Normalize
        norm = math.sqrt(w * w + x * x + y * y + z * z)

        if norm == 0.0:
            # Fallback: no rotation
            return [self.x_orient, self.y_orient, 0.0]

        w, x, y, z = w / norm, x / norm, y / norm, z / norm

        # Roll (X-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (Y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (Z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll + self.x_orient, pitch + self.y_orient, yaw]

    def servo2deg(self, mapped):
        """
        Converts servo readings to list of degrees
            - mapped: integer servo readings (list)
        """
        angles = [0 for _ in range(8)]

        # Reverse mapping rules:
        angles[0] = self.centers[0] - mapped[0]  # RHip: negate mapping
        angles[1] = mapped[1] - self.centers[1]  # LHip: same
        angles[2] = mapped[2] - self.centers[2]  # RShoulder: same
        angles[3] = mapped[3] - self.centers[3]  # LShoulder: same
        angles[4] = mapped[4] - self.centers[4]  # RKnee: same
        angles[5] = self.centers[5] - mapped[5]  # LKnee: negate mapping
        angles[6] = self.centers[6] - mapped[6]  # RAnkle: negate mapping
        angles[7] = mapped[7] - self.centers[7]  # LAnkle: same

        # Undo scaling to degrees
        for i in range(len(angles)):
            angles[i] = angles[i] / 4095.0 * 360.0

        return angles

    def deg2servo(self, angles):
        """Maps degrees to real servo positions"""
        degrees = list(angles)

        for i in range(len(degrees)):
            degrees[i] = int(degrees[i] * 4095.0 / 360.0)

        mapped = [0 for _ in range(8)]

        # Apply mapping rules:
        mapped[0] = self.centers[0] - degrees[0]  # RHip: negate
        mapped[1] = self.centers[1] + degrees[1]  # LHip: same
        mapped[2] = self.centers[2] + degrees[2]  # RShoulder: same
        mapped[3] = self.centers[3] + degrees[3]  # LShoulder: same
        mapped[4] = self.centers[4] + degrees[4]  # RKnee: same
        mapped[5] = self.centers[5] - degrees[5]  # LKnee: negate
        mapped[6] = self.centers[6] - degrees[6]  # RAnkle: negate
        mapped[7] = self.centers[7] + degrees[7]  # LAnkle: same

        return mapped
