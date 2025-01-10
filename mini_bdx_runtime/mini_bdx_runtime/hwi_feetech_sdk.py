from mini_bdx_runtime.feetech import (
    FeetechMotorsBus,
    convert_radians_to_steps,
    convert_steps_to_radians,
    configure,
)
import time
import numpy as np

# Everything is in radians here


class HWI:
    def __init__(self, usb_port="/dev/ttyACM0"):
        self.motors = {
            "right_hip_yaw": (10, "sts3215"),
            "right_hip_roll": (11, "sts3215"),
            "right_hip_pitch": (12, "sts3215"),
            "right_knee": (13, "sts3215"),
            "right_ankle": (14, "sts3215"),
            "left_hip_yaw": (20, "sts3215"),
            "left_hip_roll": (21, "sts3215"),
            "left_hip_pitch": (22, "sts3215"),
            "left_knee": (23, "sts3215"),
            "left_ankle": (24, "sts3215"),
            # "neck_pitch": (30, "sts3215"),
            # "head_pitch": (31, "sts3215"),
            # "head_yaw": (32, "sts3215"),
        }

        self.joints_offsets = {
            "right_hip_yaw": 0.0,
            "right_hip_roll": 0.0,
            "right_hip_pitch": 0.0,
            "right_knee": 0.0,
            "right_ankle": 0.0,
            "left_hip_yaw": 0.0,
            "left_hip_roll": 0.0,
            "left_hip_pitch": 0.0,
            "left_knee": 0.0,
            "left_ankle": 0.0,
            # "neck_pitch": 0.0,
            # "head_pitch": 0.0,
            # "head_yaw": 0.0,
        }

        self.joints_sign = {
            "right_hip_yaw": -1,
            "right_hip_roll": -1,
            "right_hip_pitch": 1,
            "right_knee": -1,
            "right_ankle": -1,
            "left_hip_yaw": -1,
            "left_hip_roll": -1,
            "left_hip_pitch": 1,
            "left_knee": -1,
            "left_ankle": -1,
            # "neck_pitch": -1,
            # "head_pitch": -1,
            # "head_yaw": -1,
        }

        self.init_pos = {
            "right_hip_yaw": 0.001171696610228082,
            "right_hip_roll": 0.006726989242258406,
            "right_hip_pitch": 1.0129772861831692,
            "right_knee": -1.4829304760981399,
            "right_ankle": 0.6444901047812701,
            "left_hip_yaw": -0.002853397830292128,
            "left_hip_roll": 0.01626303761810685,
            "left_hip_pitch": -1.0105624704499077,
            "left_knee": -1.4865015965817336,
            "left_ankle": 0.6504953719748071,
            # "neck_pitch": -0.17453292519943295,
            # "head_pitch": -0.17453292519943295,
            # "head_yaw": 0,
        }
        self.zero_pos = {
            "right_hip_yaw": 0,
            "right_hip_roll": 0,
            "right_hip_pitch": 0,
            "right_knee": 0,
            "right_ankle": 0,
            "left_hip_yaw": 0,
            "left_hip_roll": 0,
            "left_hip_pitch": 0,
            "left_knee": 0,
            "left_ankle": 0,
            # "neck_pitch": 0,
            # "head_pitch": 0,
            # "head_yaw": 0,
        }
        self.baudrate = 1000000

        self.motors_bus = FeetechMotorsBus(port=usb_port, motors=self.motors)
        self.motors_bus.connect()
        configure(self.motors_bus)

        # We add 180 to all dofs. They were mounted in a way that 180 degrees is the neutral position.
        self.global_offset = np.deg2rad(180)

    def enable_torque(self):
        self.motors_bus.write("Torque_Enable", 1)

    def disable_torque(self):
        self.motors_bus.write("Torque_Enable", 0)

    def turn_on(self):
        self.enable_torque()
        self.set_position_all(self.init_pos)
        # self.set_position_all(self.zero_pos)
        time.sleep(1)

    def set_position_all(self, joints_positions: dict):
        positions = np.array(list(joints_positions.values()))
        signs = np.array(list(self.joints_sign.values()))
        rads = positions * signs + self.global_offset
        rads = list(rads)

        steps = convert_radians_to_steps(rads, ["sts3215"])
        self.motors_bus.write("Goal_Position", steps)

    def get_position_all(self):

        steps = self.motors_bus.read("Present_Position", self.motors)
        rads = np.array(convert_steps_to_radians(steps, ["sts3215"]))
        return rads - self.global_offset
        # return rads * np.array(list(self.joints_sign.values()))# - self.global_offset


if __name__ == "__main__":
    hwi = HWI()
    hwi.turn_on()

    # hwi.disable_torque()
    # time.sleep(1)
    # while True:
    #     print(hwi.get_position_all())
    #     time.sleep(0.1)
