import time
from typing import List

import numpy as np

from pypot.feetech import FeetechSTS3215IO


class HWI:
    def __init__(self, usb_port="/dev/ttyACM0", baudrate=3000000):
        self.dxl_io = FeetechSTS3215IO(
            usb_port,
            baudrate=1000000,
            use_sync_read=True,
        )
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


    def set_pid(self, pid, joint_name):
        # TODO
        pass

    def set_pid_all(self, pid):
        P = np.int8(pid[0])
        I = np.int8(pid[1])
        D = np.int8(pid[2])

        self.dxl_io.set_P_coefficient({id: P for id in self.joints.values()})
        self.dxl_io.set_I_coefficient({id: I for id in self.joints.values()})
        self.dxl_io.set_D_coefficient({id: D for id in self.joints.values()})

    def get_pid_all(self):
        Ps = self.dxl_io.get_P_coefficient(self.joints.values())
        Is = self.dxl_io.get_I_coefficient(self.joints.values())
        Ds = self.dxl_io.get_D_coefficient(self.joints.values())
        print("Ps", Ps)
        print("Is", Is)
        print("Ds", Ds)

        return Ps, Is, Ds

    def turn_on(self):
        self.dxl_io.enable_torque(self.joints.values())
        self.dxl_io.set_acceleration({id: 16 for id in self.joints.values()})

        time.sleep(1)

        self.set_position_all(self.init_pos)

        time.sleep(1)

        for name, id in self.joints.items():
            if "neck" in name or "head" in name:
                self.dxl_io.set_acceleration({id: 32})
            else:
                self.dxl_io.set_acceleration({id: 254})


    def turn_off(self):
        self.dxl_io.disable_torque(self.joints.values())

    def set_position_all(self, joints_positions):
        """
        joints_positions is a dictionary with joint names as keys and joint positions as values
        Warning: expects radians
        """
        ids_positions = {
            self.joints[joint]: np.rad2deg(position + self.joints_offsets[joint])
            for joint, position in joints_positions.items()
        }

        self.dxl_io.set_goal_position(ids_positions)

    def set_position(self, joint_name, position):
        self.dxl_io.set_goal_position({self.joints[joint_name]: np.rad2deg(-position)})

    def get_present_positions(self):
        """
        Returns the present positions in radians
        """
        present_positions = np.deg2rad(self.dxl_io.get_present_position(self.joints.values()))
        present_positions = [pos - self.joints_offsets[joint] for joint, pos in zip(self.joints.keys(), present_positions)]
        return np.array(np.around(present_positions, 3))


    def get_present_velocities(self, rad_s=True):
        """
        Returns the present velocities in rad/s (default) or rev/min
        """
        # rev/min
        present_velocities = np.array(
            self.dxl_io.get_present_speed(self.joints.values())
        )
        if rad_s:
            present_velocities = (2 * np.pi * present_velocities) / 60  # rad/s
        return np.array(np.around(present_velocities, 3))
