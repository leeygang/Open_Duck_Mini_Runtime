from mini_bdx_runtime.hwi_feetech_pwm_control import HWI
import time
import numpy as np

joints_order = [
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "left_antenna",
    "right_antenna",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
]


def make_action_dict(action, joints_order):
    action_dict = {}
    for i, a in enumerate(action):
        if "antenna" not in joints_order[i]:
            action_dict[joints_order[i]] = a

    return action_dict

hwi = HWI("/dev/ttyACM0")
hwi.turn_on()

time.sleep(1)

starting_pos_rad = np.array(hwi.get_present_positions())
freq = 50
A = np.deg2rad(10)
F = 0.1
times = []
start = time.time()
while True:
    if time.time() - start > 3:
        break
    action = starting_pos_rad + A * np.sin(2*np.pi*F*time.time())
    action_dict = make_action_dict(action, joints_order)
    s = time.time()
    hwi.set_position_all(action_dict)
    took = time.time() - s
    print(f"set_position_all took {took}s")
    times.append(took)
    
    time.sleep(1/freq)

print(f"Average set_position_all took {np.mean(times)}s")