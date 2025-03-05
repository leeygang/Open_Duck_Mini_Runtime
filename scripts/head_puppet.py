"""
Sets up the robot in init position, you control the head with the xbox controller
"""


# from pypot.feetech import FeetechSTS3215IO
# from mini_bdx_runtime.hwi_feetech_pypot import HWI
from mini_bdx_runtime.rustypot_position_hwi import HWI
from mini_bdx_runtime.eyes import Eyes
import time
import pygame
import numpy as np

eyes = Eyes()
hwi = HWI()

kps = [16] * 14
kds = [0] * 14

hwi.set_kps(kps)
hwi.set_kds(kds)
hwi.turn_on()


pygame.init()
_p1 = pygame.joystick.Joystick(0)
_p1.init()
print(f"Loaded joystick with {_p1.get_numaxes()} axes.")


limits = {
    "neck_pitch" : [-20, 60],
    "head_pitch" : [-60, 45],
    "head_yaw" : [-60, 60],
    "head_roll" : [-20, 20],
}

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

    # print("l_x", l_x)
    # print("l_y", l_y)
    # print("r_x", r_x)
    # print("r_y", r_y)
    # print("===")


    head_yaw_deg = l_x * (limits["head_yaw"][1] - limits["head_yaw"][0]) / 2 + (limits["head_yaw"][1] + limits["head_yaw"][0]) / 2
    head_yaw_pos_rad = np.deg2rad(head_yaw_deg)

    head_roll_deg = r_x * (limits["head_roll"][1] - limits["head_roll"][0]) / 2 + (limits["head_roll"][1] + limits["head_roll"][0]) / 2
    head_roll_pos_rad = np.deg2rad(head_roll_deg)


    head_pitch_deg = l_y * (limits["head_pitch"][1] - limits["head_pitch"][0]) / 2 + (limits["head_pitch"][1] + limits["head_pitch"][0]) / 2
    head_pitch_pos_rad = np.deg2rad(head_pitch_deg)

    neck_pitch_deg = -r_y * (limits["neck_pitch"][1] - limits["neck_pitch"][0]) / 2 + (limits["neck_pitch"][1] + limits["neck_pitch"][0]) / 2
    neck_pitch_pos_rad = np.deg2rad(neck_pitch_deg)


    # Convert from degrees to radians
    # print(head_yaw_deg)

    hwi.set_position("head_yaw", head_yaw_pos_rad)
    hwi.set_position("head_roll", head_roll_pos_rad)
    hwi.set_position("head_pitch", head_pitch_pos_rad)
    hwi.set_position("neck_pitch", neck_pitch_pos_rad)


    pygame.event.pump()  # process event queue
    time.sleep(1/60)
