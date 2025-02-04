from mini_bdx_runtime.hwi_feetech_pwm_control import HWI
import numpy as np
import time
import RPi.GPIO as GPIO

LEFT_FOOT_PIN = 22
RIGHT_FOOT_PIN = 27

hwi = HWI()
hwi.turn_on()


def get_feet_contacts():
    left = False
    right = False
    if GPIO.input(LEFT_FOOT_PIN) == GPIO.LOW:
        left = True
    if GPIO.input(RIGHT_FOOT_PIN) == GPIO.LOW:
        right = True
    return np.array([left, right])


control_freq = 50
zero_pos = hwi.init_pos.copy()
A = 0.1
F = 0.5
try:
    s = time.time()
    while True:
        s = time.time()

        target = A * np.sin(2 * np.pi * F * time.time())
        for name in zero_pos.keys():
            zero_pos[name] = target + hwi.init_pos[name]

        hwi.set_position_all(zero_pos)
        feet_contacts = get_feet_contacts()

        took = time.time() - s
        time.sleep(max(0, 1 / control_freq - took))
except KeyboardInterrupt:
    hwi.freeze()

    time.sleep(2)
    exit()


hwi.freeze()
time.sleep(2)
