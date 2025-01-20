import time
from typing import List

import numpy as np

# from pypot.feetech import FeetechSTS3215IO
from mini_bdx_runtime.feetech_pwm_control import FeetechPWMControl


class HWI:
    def __init__(self, usb_port="/dev/ttyACM0"):

        # Order matters here
        self.joints = {
            "left_hip_yaw": 20,
            "left_hip_roll": 21,
            "left_hip_pitch": 22,
            "left_knee": 23,
            "left_ankle": 24,
            "neck_pitch": 30,
            "head_pitch": 31,
            "head_yaw": 32,
            "head_roll": 33,
            # "left_antenna": None,
            # "right_antenna": None,
            "right_hip_yaw": 10,
            "right_hip_roll": 11,
            "right_hip_pitch": 12,
            "right_knee": 13,
            "right_ankle": 14,
        }

        self.zero_pos = {
            "left_hip_yaw": 0,
            "left_hip_roll": 0,
            "left_hip_pitch": 0,
            "left_knee": 0,
            "left_ankle": 0,
            "neck_pitch": 0,
            "head_pitch": 0,
            "head_yaw": 0,
            "head_roll": 0,
            # "left_antenna":0,
            # "right_antenna":0,
            "right_hip_yaw": 0,
            "right_hip_roll": 0,
            "right_hip_pitch": 0,
            "right_knee": 0,
            "right_ankle": 0,
        }

        self.init_pos = {
            "left_hip_yaw": 0.002,
            "left_hip_roll": 0.053,
            "left_hip_pitch": -0.63,
            "left_knee": 1.368,
            "left_ankle": -0.784,
            "neck_pitch": 0.002,
            "head_pitch": 0.0,
            "head_yaw": 0,
            "head_roll": 0,
            # "left_antenna": 0,
            # "right_antenna": 0,
            "right_hip_yaw": -0.003,
            "right_hip_roll": -0.065,
            "right_hip_pitch": 0.635,
            "right_knee": 1.379,
            "right_ankle": -0.796,
        }

        # self.init_pos = self.zero_pos  # TODO REMOVE

        self.joints_offsets = {
            "left_hip_yaw": 0.07,
            "left_hip_roll": -0.1,
            "left_hip_pitch": 0.0,
            "left_knee": 0.05,
            "left_ankle": -0.1,
            "neck_pitch": 0.1,
            "head_pitch": 0.1,
            "head_yaw": 0,
            "head_roll": 0.1,
            # "left_antenna": 0,
            # "right_antenna": 0,
            "right_hip_yaw": -0.15,
            "right_hip_roll": 0.07,
            "right_hip_pitch": 0.05,
            "right_knee": -0.05,
            "right_ankle": -0.08,
        }

        self.control = FeetechPWMControl(
            ids=list(self.joints.values()),
            init_pos_rad=list(self.init_pos.values()),
            usb_port=usb_port,
        )

        self.kps = np.ones(len(self.joints)) * 32  # default kp
        self.low_torque_kps = np.ones(len(self.joints)) * 8  # default kp

    def set_kps(self, kps):
        self.kps = kps
        self.control.set_kps(self.kps)

    def turn_on(self):
        self.control.set_kps(self.low_torque_kps)
        # self.control.enable_torque()
        print("turn on : low KPS set")
        time.sleep(1)

        self.set_position_all(self.init_pos)
        print("turn on : init pos set")

        time.sleep(1)

        self.control.set_kps(self.kps)
        print("turn on : high kps")

    def turn_off(self):
        self.control.disable_torque()

    def freeze(self):
        self.control.freeze()


    def set_position_all(self, joints_positions):
        """
        joints_positions is a dictionary with joint names as keys and joint positions as values
        Warning: expects radians
        """
        ids_positions = {
            self.joints[joint]: np.rad2deg(position + self.joints_offsets[joint])
            for joint, position in joints_positions.items()
        }

        self.control.goal_positions = list(ids_positions.values())

    def get_present_positions(self):
        """
        Returns the present positions in radians
        """
        present_positions = np.deg2rad(
            self.control.io.get_present_position(self.joints.values())
        )
        present_positions = [
            pos - self.joints_offsets[joint]
            for joint, pos in zip(self.joints.keys(), present_positions)
        ]
        return np.array(np.around(present_positions, 3))

    def get_present_velocities(self, rad_s=True):
        """
        Returns the present velocities in rad/s (default) or rev/min
        """
        # rev/min
        present_velocities = np.array(
            self.control.io.get_present_speed(self.joints.values())
        )
        if rad_s:
            present_velocities = (2 * np.pi * present_velocities) / 60  # rad/s
        return np.array(np.around(present_velocities, 3))
