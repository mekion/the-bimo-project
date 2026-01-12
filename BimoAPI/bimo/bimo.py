# Copyright (c) 2025, Mekion
# SPDX-License-Identifier: Apache-2.0
"""Core Bimo Robotics Kit control class"""

from time import sleep
import serial
import struct
import math


class Bimo():
    """Bimo Robotics Kit control class"""

    def __init__(self, baudrate=115200, timeout=0.2):
        # MCU serial comms
        self.mcu = self.__connect(baudrate, timeout)

        # Servos
        self.servo_max = [90, 90, 90, 12, 140, 140, 93, 93]
        self.servo_min = [-90, -90, -12, -90, 0, 0, -93, -93]
        self.sit_pose = [-45, -45, 0, 0, 140, 140, 93, 93]
        self.stand_pose = [-30, -30, 0, 0, 60, 60, 30, 30]
        self.centers = [1509, 2587, 2048, 2048, 456, 3640, 2048, 2048]

        # IMU calibration offsets
        self.x_orient = 0
        self.y_orient = 0

    # UTILITY
    def initialize(self):
        """Initializes robot from sitting position"""
        self.send_positions(self.sit_pose)
        sleep(2)  # Allows robot to settle
        self.update_imu_offsets()

        print("Initialization complete")

    def calibrate_robot(self):
        """Calibrates Servos"""
        print("Palce robot on flat gorund and align, legs and head, using the provided guides.")
        input("Press Enter to continue...")

        self.calibrate()

        # Calibrates ankles on centered position
        print("Lift robot from ground and remove the leg guide. Ankles will move and calibrate.")
        input("Press Enter to continue...")

        curr_pose = [2048 for _ in range(8)]
        ankle_diff = int(92.54 / 360 * 4095)

        curr_pose[6] += ankle_diff
        curr_pose[7] -= ankle_diff

        self.send_positions(self.servo2deg(curr_pose))
        sleep(3)
        self.calibrate()

        print("Servo calibration successful! Place robot on the ground in sitting pose.")
        input("Press Enter to continue...")

    def update_imu_offsets(self):
        """Updates IMU offset"""
        imu, _, _ = self.request_state_data()
        euler = self.quaternion_to_euler(imu[:4])

        self.x_orient = -euler[0]
        self.y_orient = -euler[1]

    # COMMS
    def __connect(self, baudrate, timeout):
        # Tries to connect to MCU 5 times
        for attempt in range(5):
            mcu = None
            for port in range(5):
                try:
                    mcu = serial.Serial(f"/dev/ttyACM{port}", baudrate=baudrate, timeout=timeout)
                    return mcu

                except Exception:
                    continue

            # Waits 100ms before trying again
            sleep(0.1)

        # Raises exception if no connection established
        raise Exception("ERROR: Could not connect to MCU!")

    def request_state_data(self):
        """Returns robot state based on sensor data: IMU, Distance, Battery/Power Voltage"""
        # Asks for state data, Message [Byte Count, Request Code]
        self.mcu.write(struct.pack("2i", *[4, 1]))

        # Processes message data
        data = struct.unpack("12f", self.mcu.read(48))

        # Returns IMU, Distance, Battery
        return data[:7], data[7:11], data[-1]

    def request_positions(self):
        """Returns robot current servo positions"""
        # Asks for state data, Message [Byte Count, Request Code]
        self.mcu.write(struct.pack("2i", *[4, 3]))

        # Processes message data
        data = list(struct.unpack("8i", self.mcu.read(32)))

        return self.servo2deg(data)

    def send_positions(self, actions):
        """Sends list of actions in degrees to MCU"""
        act = self.deg2servo(actions)

        # Message [Byte Count, Actions * 4 Bytes]
        self.mcu.write(struct.pack("9i", *[32] + act))

    def calibrate(self):
        """Sends calibration command"""
        self.mcu.write(struct.pack("2i", *[4, 4]))

    def available(self):
        """Returns True if MCU ready."""
        for i in range(5):
            status = None

            # Ask if available, Message [Byte Count, Request Code]
            self.mcu.write(struct.pack("2i", *[4, 2]))

            # MCU availability status
            status = struct.unpack("i", self.mcu.read(4))

            if status == 1:
                return True

            sleep(0.1)

        return False

    def port(self):
        return self.mcu.port

    # HELPERS
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
            raise ValueError("values, mins, and maxs must have the same length.")

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
