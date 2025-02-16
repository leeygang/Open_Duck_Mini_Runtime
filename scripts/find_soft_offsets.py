"""
Find the offsets to set in self.joints_offsets in hwi_feetech_pwm_control.py
"""

from mini_bdx_runtime.hwi_feetech_pwm_control import HWI

hwi = HWI()
hwi.joints_offsets = {
    "left_hip_yaw": 0,
    "left_hip_roll": 0,
    "left_hip_pitch": 0,
    "left_knee": 0,
    "left_ankle": 0,
    "neck_pitch": 0,
    "head_pitch": 0,
    "head_yaw": 0,
    "head_roll": 0,
    "right_hip_yaw": 0,
    "right_hip_roll": 0,
    "right_hip_pitch": 0,
    "right_knee": 0,
    "right_ankle": 0,
}

hwi.init_pos = hwi.zero_pos

try:
    for joint_name in hwi.joints.keys():
        print(f" === Setting up {joint_name} === ")
        input("Press any key to set torque on for this motor. It will go to its zero position.")
        

except KeyboardInterrupt:
    hwi.turn_off()