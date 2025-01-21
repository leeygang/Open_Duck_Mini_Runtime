from pypot.feetech import FeetechSTS3215IO
import time
import numpy as np
from threading import Thread


class FeetechPWMControl:
    def __init__(self, ids, init_pos_rad, usb_port="/dev/ttyACM0"):
        self.io = FeetechSTS3215IO(
            usb_port,
            baudrate=1000000,
            use_sync_read=True,
        )
        self.ids = ids

        # TODO remove this ?
        # self.io.enable_torque(self.ids)
        # self.io.set_mode({id: 0 for id in self.ids})
        # self.io.set_goal_position({id: 0 for id in self.ids})
        # time.sleep(1)
        # self.io.disable_torque({id: 0 for id in self.ids})
        # exit()

        self.io.set_mode({id: 2 for id in self.ids})
        self.kps = np.ones(len(self.ids)) * 32  # default kp

        self.init_pos_rad = init_pos_rad
        self.init_pos_deg = np.rad2deg(self.init_pos_rad)

        self.control_freq = 100  # Hz
        self.goal_positions = [0] * len(self.ids)
        self.goal_positions = self.init_pos_deg
        self.present_positions = [0] * len(self.ids)
        self.last_positions = [0] * len(self.ids)
        self.present_speeds = [0] * len(self.ids)
        self.speed_decimation = 2
        self.speed_decimation_index = 0

        Thread(target=self.update, daemon=True).start()

    def set_kps(self, kps):
        self.kps = np.array(kps)

    def disable_torque(self):
        self.io.set_mode({id: 0 for id in self.ids})
        self.io.disable_torque(self.ids)

    # def enable_torque(self):
    #     self.io.enable_torque(self.ids)
    #     self.io.set_mode({id: 2 for id in self.ids})

    def freeze(self):
        present_position = list(self.io.get_present_position(self.ids))
        print(present_position)
        self.io.set_goal_position(
            {id: present_position[i] for i, id in enumerate(self.ids)}
        )
        self.io.set_mode({id: 0 for id in self.ids})
        self.io.enable_torque(self.ids)

    def update(self):
        while True:
            s = time.time()
            self.present_positions = self.io.get_present_position(self.ids)
            errors = np.array(self.goal_positions) - np.array(self.present_positions)

            pwms = self.kps * errors
            # pwms *= 10
            pwms = np.int16(pwms)

            pwm_magnitudes = abs(pwms)
            for i in range(len(pwm_magnitudes)):
                if pwm_magnitudes[i] >= 2**10:
                    pwm_magnitudes[i] = (2**10) - 1

            # direction_bits = 1 if pwms >= 0 else 0
            direction_bits = [1 if pwm >= 0 else 0 for pwm in pwms]

            # goal_times = (direction_bits << 10) | pwm_magnitudes
            goal_times = [
                (direction_bits[i] << 10) | pwm_magnitudes[i]
                for i in range(len(pwm_magnitudes))
            ]

            self.io.set_goal_time({id: goal_times[i] for i, id in enumerate(self.ids)})
            print(self.speed_decimation_index)
            if self.speed_decimation_index % self.speed_decimation == 0:
                self.present_speeds = (
                    np.array(self.present_positions) - np.array(self.last_positions)
                ) / (1 / self.control_freq)
                self.last_positions = np.array(self.present_positions).copy()
                print("PRESENT SPEEDS : ", self.present_speeds)
            took = time.time() - s
            time.sleep(max(0, (1 / self.control_freq - took)))
            self.speed_decimation_index


if __name__ == "__main__":
    ids = [1, 2]
    pwm_control = FeetechPWMControl(ids)
    pwm_control.enable_torque()
    pwm_control.set_kps([32, 32])

    pwm_control.goal_positions = [-90, -90]
    try:
        while True:
            pwm_control.goal_positions = [90, 90]
            time.sleep(1)
            pwm_control.goal_positions = [-90, -90]
            time.sleep(1)
    except KeyboardInterrupt:
        pwm_control.disable_torque()
