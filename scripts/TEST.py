import pickle
from mini_bdx_runtime.hwi_feetech_pwm_control import HWI
import numpy as np
import time

hwi = HWI()
hwi.turn_on()
# while True:
#     time.sleep(0.01)
# exit()
time.sleep(1)


def add_fake_antennas(pos):
    # takes in position without antennas, adds position 0 for both antennas at the right place
    assert len(pos) == 14
    pos_with_antennas = np.insert(pos, 9, [0, 0])
    return np.array(pos_with_antennas)


def get_obs(prev_action):

    commands = [0, 0, 0]

    dof_pos = hwi.get_present_positions()  # rad
    dof_vel = hwi.get_present_velocities()  # rad/s

    dof_pos = add_fake_antennas(dof_pos)
    dof_vel = add_fake_antennas(dof_vel)

    dof_pos_scaled = list(dof_pos * 1)
    dof_vel_scaled = list(dof_vel * 1)

    projected_gravity = [0, 0, 0]
    feet_contacts = [0, 0]

    obs = np.concatenate(
        [
            projected_gravity,
            dof_pos_scaled,
            dof_vel_scaled,
            feet_contacts,
            prev_action,
            commands,
        ]
    )

    return obs


zero_pos = hwi.zero_pos.copy()
id = 2
name = list(hwi.joints.keys())[id]
A = 0.3
F = 2.0
control_freq = 50
saved_obs = []
all_s = time.time()
try:
    s = time.time()
    while True:
        s = time.time()

        target = A * np.sin(2 * np.pi * F * time.time())
        zero_pos[name] = target

        hwi.set_position_all(zero_pos)
        prev_pos = add_fake_antennas(list(zero_pos.values()))

        obs = get_obs(prev_pos)

        saved_obs.append(obs)

        if time.time() - all_s > 10:
            break

        took = time.time() - s
        time.sleep(max(0, 1 / control_freq - took))
except KeyboardInterrupt:
    hwi.freeze()

    time.sleep(2)
    exit()


hwi.freeze()
time.sleep(2)


pickle.dump(saved_obs, open("robot_saved_obs.pkl", "wb"))
