from mini_bdx_runtime.rustypot_position_hwi import HWI

joints_offsets = {
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

hwi = HWI(joints_offsets)
hwi.turn_off()
