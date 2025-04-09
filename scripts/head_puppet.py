"""
Sets up the robot in init position, you control the head with the xbox controller
"""

from mini_bdx_runtime.rustypot_position_hwi import HWI
from mini_bdx_runtime.eyes import Eyes
import time
import pygame
import numpy as np


from mini_bdx_runtime.sounds import Sounds
from mini_bdx_runtime.antennas import Antennas
from mini_bdx_runtime.projector import Projector

sounds = Sounds(volume=1.0, sound_directory="../mini_bdx_runtime/assets/")
antennas = Antennas()

eyes = Eyes()
projector = Projector()

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

kps = [8] * 14
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
left_trigger = 0
right_trigger = 0
while True:
    X_pressed = False
    A_pressed = False

    for event in pygame.event.get():
        l_x = round(_p1.get_axis(0), 3)
        l_y = round(_p1.get_axis(1), 3)
        r_x = round(_p1.get_axis(2), 3)
        r_y = round(_p1.get_axis(3), 3)


        right_trigger = np.around((_p1.get_axis(4) + 1) / 2, 3)
        left_trigger = np.around((_p1.get_axis(5) + 1) / 2, 3)

        if _p1.get_button(0):  # A button
            A_pressed = True


        if _p1.get_button(3):  # X button
            X_pressed = True

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

    hwi.set_position("head_yaw", head_yaw_pos_rad)
    hwi.set_position("head_roll", head_roll_pos_rad)
    hwi.set_position("head_pitch", head_pitch_pos_rad)
    hwi.set_position("neck_pitch", neck_pitch_pos_rad)


    antennas.set_position_left(right_trigger)
    antennas.set_position_right(left_trigger)

    if X_pressed:
        sounds.play_random_sound()
    if A_pressed:
        projector.switch()


    pygame.event.pump()  # process event queue
    time.sleep(1/60)
