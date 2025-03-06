import adafruit_bno055
import board
import busio
import numpy as np

from queue import Queue
from threading import Thread
import time


# TODO filter spikes
class Imu:
    def __init__(self, sampling_freq, user_pitch_bias=0, calibration=False):
        self.sampling_freq = sampling_freq
        self.user_pitch_bias = user_pitch_bias
        self.nominal_pitch_bias = 25

        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(i2c)

        # self.imu.mode = adafruit_bno055.IMUPLUS_MODE
        self.imu.mode = adafruit_bno055.ACCGYRO_MODE
        # self.imu.mode = adafruit_bno055.GYRONLY_MODE
        # self.imu.mode = adafruit_bno055.NDOF_MODE
        # self.imu.mode = adafruit_bno055.NDOF_FMC_OFF_MODE

        self.pitch_bias = self.nominal_pitch_bias + self.user_pitch_bias

        # # calibrate imu
        # calibrated = self.imu.calibration_status[1] == 3
        # print("Calibrating Gyro...")
        # while not calibrated:
        #     calibrated = self.imu.calibration_status[1] == 3
        #     print("Calibration status: ", self.imu.calibration_status[1])
        #     time.sleep(0.1)

        self.last_imu_data = [0, 0, 0, 0]
        self.last_imu_data = {
            "gyro": [0, 0, 0],
            "accelero": [0, 0, 0],
        }
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def convert_axes(self, euler):
        euler = [np.pi + euler[1], euler[0], euler[2]]
        return euler

    def imu_worker(self):
        while True:
            s = time.time()
            try:
                gyro = np.array(self.imu.gyro).copy()
                accelero = np.array(self.imu.acceleration).copy()
            except Exception as e:
                print("[IMU]:", e)
                continue

            # Converting to correct axes
            # euler = self.convert_axes(euler)
            # euler[1] -= np.deg2rad(self.pitch_bias)
            # euler[2] = 0  # ignoring yaw

            # gives scalar last, which is what isaac wants
            # final_orientation_quat = R.from_euler("xyz", euler).as_quat()

            print(gyro)
            print(accelero)
            

            data = {
                "gyro": gyro,
                "accelero": accelero,
            }

            self.imu_queue.put(data)
            took = time.time() - s
            time.sleep(max(0, 1 / self.sampling_freq - took))

    def get_data(self):
        try:
            self.last_imu_data = self.imu_queue.get(False)  # non blocking
        except Exception:
            pass

        return self.last_imu_data


if __name__ == "__main__":
    imu = Imu(50)
    while True:
        data = imu.get_data()
        print("gyro", np.around(data["gyro"], 3))
        print("accelero", np.around(data["accelero"], 3))
        print("---")
        time.sleep(1 / 25)
