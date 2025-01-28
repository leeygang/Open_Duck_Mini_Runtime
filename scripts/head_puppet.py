"""
Sets up the robot in init position, you control the head with the xbox controller
"""


from pypot.feetech import FeetechSTS3215IO
from mini_bdx_runtime.hwi_feetech_pypot import HWI
import time
import pygame
import numpy as np
from mini_bdx_runtime.imu import Imu


from queue import Queue
from threading import Thread

hwi = HWI()
hwi.turn_on()

imu = Imu(50, -18)

pygame.init()
_p1 = pygame.joystick.Joystick(0)
_p1.init()
print(f"Loaded joystick with {_p1.get_numaxes()} axes.")


limits = {
    "neck_pitch": [-40, 20],
    "head_pitch": [-45, 45],
    "head_yaw": [-60, 60],
    "head_roll": [-20, 20],
}


cmd_queue = Queue(maxsize=1)


def commands_worker():
    global _p1, cmd_queue
    l_x = 0
    l_y = 0
    r_x = 0
    r_y = 0
    while True:
        for event in pygame.event.get():
            l_x = round(_p1.get_axis(0), 3)
            l_y = round(_p1.get_axis(1), 3)
            r_x = round(_p1.get_axis(2), 3)
            r_y = round(_p1.get_axis(3), 3)
        pygame.event.pump()  # process event queue
        cmd_queue.put([l_x, l_y, r_x, r_y])
        time.sleep(1 / 30)


last_command = [0, 0, 0, 0]


def get_command():
    global last_command
    try:
        last_command = cmd_queue.get(False)  # non blocking
    except Exception:
        pass

    return last_command


Thread(target=commands_worker, daemon=True).start()
while True:
    l_x, l_y, r_x, r_y = get_command()
    head_yaw_deg = (
        l_x * (limits["head_yaw"][1] - limits["head_yaw"][0]) / 2
        + (limits["head_yaw"][1] + limits["head_yaw"][0]) / 2
    )
    head_yaw_pos_rad = np.deg2rad(head_yaw_deg)

    head_roll_deg = (
        r_x * (limits["head_roll"][1] - limits["head_roll"][0]) / 2
        + (limits["head_roll"][1] + limits["head_roll"][0]) / 2
    )
    head_roll_pos_rad = np.deg2rad(head_roll_deg)

    head_pitch_deg = (
        l_y * (limits["head_pitch"][1] - limits["head_pitch"][0]) / 2
        + (limits["head_pitch"][1] + limits["head_pitch"][0]) / 2
    )
    head_pitch_pos_rad = np.deg2rad(head_pitch_deg)

    neck_pitch_deg = (
        -r_y * (limits["neck_pitch"][1] - limits["neck_pitch"][0]) / 2
        + (limits["neck_pitch"][1] + limits["neck_pitch"][0]) / 2
    )
    neck_pitch_pos_rad = np.deg2rad(neck_pitch_deg)

    euler = np.rad2deg(imu.get_data(euler=True))
    print(euler)

    hwi.set_position("head_yaw", head_yaw_pos_rad)
    hwi.set_position("head_roll", head_roll_pos_rad)
    hwi.set_position("head_pitch", head_pitch_pos_rad)
    hwi.set_position("neck_pitch", neck_pitch_pos_rad)

    time.sleep(1 / 60)
