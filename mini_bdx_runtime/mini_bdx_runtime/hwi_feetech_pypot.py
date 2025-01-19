import time
from typing import List

import numpy as np

# from mini_bdx_runtime.io_330 import Dxl330IO
from pypot.feetech import FeetechSTS3215IO


class HWI:
    def __init__(self, usb_port="/dev/ttyACM0", baudrate=3000000):
        # self.dxl_io = Dxl330IO(usb_port, baudrate=baudrate, use_sync_read=True)
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

        # init_pos = np.array(
        # [
        #     0.002,
        #     0.053,
        #     -0.63,
        #     1.368,
        #     -0.784,
        #     0.002,
        #     0,
        #     0,
        #     0,
        #     0,
        #     0,
        #     -0.003,
        #     -0.065,
        #     0.635,
        #     1.379,
        #     -0.796,
        # ]
        # )

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
            "right_hip_roll": 0.12,
            "right_hip_pitch": 0.05,
            "right_knee": -0.05,
            "right_ankle": -0.08,
        }

        self.joints_sign = {
            "left_hip_yaw": -1,
            "left_hip_roll": -1,  # was -1
            "left_hip_pitch": -1,
            "left_knee": -1,
            "left_ankle": -1,
            "neck_pitch": -1,
            "head_pitch": -1,
            "head_yaw": -1,
            "head_roll": -1,
            # "left_antenna": -1,
            # "right_antenna": -1,
            "right_hip_yaw": -1,
            "right_hip_roll": -1,  # was -1
            "right_hip_pitch": -1,
            "right_knee": -1,
            "right_ankle": -1,
        }

        # self.init_pos = {
        #     joint: position - offset
        #     for joint, position, offset in zip(
        #         self.init_pos.keys(),
        #         self.init_pos.values(),
        #         self.joints_offsets.values(),
        #     )
        # }

        # current based position : 0x5
        # self.dxl_io.set_operating_mode({id: 0x3 for id in self.joints.values()})

    def set_pid(self, pid, joint_name):
        self.dxl_io.set_pid_gain({self.joints[joint_name]: pid})

    def set_pid_all(self, pid):
        self.dxl_io.set_pid_gain({id: pid for id in self.joints.values()})

    def set_low_torque(self):
        self.dxl_io.set_pid_gain({id: [100, 0, 0] for id in self.joints.values()})

    def set_high_torque(self):
        # https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#position-pid-gain80-82-84-feedforward-1st2nd-gains88-90
        # 128 P factor
        # 16 D factor
        self.dxl_io.set_pid_gain(
            {id: [10 * 128, 0, int(0.5 * 16)] for id in self.joints.values()}
        )
        for name in ["neck_pitch", "head_pitch", "head_yaw"]:
            self.dxl_io.set_pid_gain({self.joints[name]: [150, 0, 0]})

    def turn_on(self):
        # self.set_low_torque()
        self.dxl_io.enable_torque(self.joints.values())
        self.dxl_io.set_acceleration({id: 16 for id in self.joints.values()})
        time.sleep(1)
        # self.goto_zero()
        self.set_position_all(self.init_pos)
        time.sleep(1)
        for name, id in self.joints.items():
            if "neck" in name or "head" in name:
                self.dxl_io.set_acceleration({id: 32})
            else:
                self.dxl_io.set_acceleration({id: 254})

        # self.dxl_io.set_acceleration({id: 254 for id in self.joints.values()})

    def turn_off(self):
        self.dxl_io.disable_torque(self.joints.values())

    def goto_zero(self):
        goal_dict = {joint: 0 for joint in self.joints.keys()}

        self.set_position_all(goal_dict)

    def set_position_all(self, joints_positions):
        """
        joints_positions is a dictionary with joint names as keys and joint positions as values
        Warning: expects radians
        """
        ids_positions = {
            self.joints[joint]: self.joints_sign[joint]
            * np.rad2deg(-position - self.joints_offsets[joint])
            for joint, position in joints_positions.items()
        }

        # print(ids_positions)
        self.dxl_io.set_goal_position(ids_positions)

    def set_position(self, joint_name, position):
        self.dxl_io.set_goal_position({self.joints[joint_name]: np.rad2deg(-position)})

    def get_present_current(self, joint_name):
        return self.dxl_io.get_present_current([self.joints[joint_name]])[0]

    def get_voltage_all(self):
        return self.dxl_io.get_present_input_voltage(self.joints.values())

    def get_present_input_voltage(self, joint_name):
        return self.dxl_io.get_present_input_voltage([self.joints[joint_name]])[0]

    def get_current_all(self):
        return self.dxl_io.get_present_current(self.joints.values())

    def get_goal_current(self, joint_name):
        return self.dxl_io.get_goal_current([self.joints[joint_name]])[0]

    def get_current_limit(self, joint_name):
        return self.dxl_io.get_current_limit([self.joints[joint_name]])[0]

    def get_present_positions(self):
        present_position = list(
            np.around(
                np.deg2rad((self.dxl_io.get_present_position(self.joints.values()))), 3
            )
        )
        factor = np.ones(len(present_position)) * -1
        return present_position * factor

    def get_present_velocities(self, rad_s=True) -> List[float]:
        """
        Returns the present velocities in rad/s or rev/min
        """
        # rev/min
        present_velocities = np.array(
            self.dxl_io.get_present_velocity(self.joints.values())
        )
        if rad_s:
            present_velocities = (2 * np.pi * present_velocities) / 60  # rad/s

        factor = np.ones(len(present_velocities)) * -1
        return list(present_velocities * factor)

    def get_operating_modes(self):
        return self.dxl_io.get_operating_mode(self.joints.values())
