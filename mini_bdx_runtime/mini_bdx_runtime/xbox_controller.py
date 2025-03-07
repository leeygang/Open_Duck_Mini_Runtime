import pygame
from threading import Thread
from queue import Queue
import time
import numpy as np


X_RANGE = [-0.1, 0.15]
Y_RANGE = [-0.2, 0.2]
YAW_RANGE = [-0.5, 0.5]
YAW_RANGE = [-1.0, 1.0]

# rads
NECK_PITCH_RANGE = ([-0.34, 1.1],)
HEAD_PITCH_RANGE = ([-0.78, 0.78],)
HEAD_YAW_RANGE = ([-2.7, 2.7],)
HEAD_ROLL_RANGE = ([-0.5, 0.5],)


class XBoxController:
    def __init__(self, command_freq):
        self.command_freq = command_freq
        self.last_commands = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pygame.init()
        self.p1 = pygame.joystick.Joystick(0)
        self.p1.init()
        print(f"Loaded joystick with {self.p1.get_numaxes()} axes.")
        self.cmd_queue = Queue(maxsize=1)
        Thread(target=self.commands_worker, daemon=True).start()

    def commands_worker(self):
        while True:
            self.cmd_queue.put(self.get_commands())
            time.sleep(1 / self.command_freq)

    def get_commands(self):
        A_pressed = False
        last_commands = self.last_commands
        for event in pygame.event.get():
            lin_vel_y = -1 * self.p1.get_axis(0)
            lin_vel_x = -1 * self.p1.get_axis(1)
            ang_vel = -1 * self.p1.get_axis(2)
            if lin_vel_x >= 0:
                lin_vel_x *= np.abs(X_RANGE[1])
            else:
                lin_vel_x *= np.abs(X_RANGE[0])

            if lin_vel_y >= 0:
                lin_vel_y *= np.abs(Y_RANGE[1])
            else:
                lin_vel_y *= np.abs(Y_RANGE[0])

            if ang_vel >= 0:
                ang_vel *= np.abs(YAW_RANGE[1])
            else:
                ang_vel *= np.abs(YAW_RANGE[0])

            last_commands[0] = lin_vel_x
            last_commands[1] = lin_vel_y
            last_commands[2] = ang_vel

            if self.p1.get_button(0):  # A button
                A_pressed = True

        pygame.event.pump()  # process event queue

        return np.around(last_commands, 3), A_pressed

    def get_last_command(self):
        A_pressed = False
        try:
            self.last_commands, A_pressed = self.cmd_queue.get(False)  # non blocking
        except Exception:
            pass

        return self.last_commands, A_pressed
