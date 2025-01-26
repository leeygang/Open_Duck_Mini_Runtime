import adafruit_bno055
import board
import busio
import numpy as np

from queue import Queue
from threading import Thread
import time
from scipy.spatial.transform import Rotation as R


class Imu:
    def __init__(self, sampling_freq, pitch_bias=0, calibration=False):
        self.sampling_freq = sampling_freq
        self.pitch_bias = pitch_bias

        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(i2c)

        self.imu.mode = adafruit_bno055.IMUPLUS_MODE
        # self.imu.mode = adafruit_bno055.GYRONLY_MODE
        # self.imu.mode = adafruit_bno055.NDOF_MODE

        if calibration:
            print("press ctrl+c when you are satisfied with the calibration")
            print("Calibrating...")
            try:
                while True:
                    print(self.imu.calibration_status)
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("Calibration done.")
                exit()

        # # calibrate imu
        # calibrated = self.imu.calibration_status[1] == 3
        # print("Calibrating Gyro...")
        # while not calibrated:
        #     calibrated = self.imu.calibration_status[1] == 3
        #     time.sleep(0.01)

        # self.zero_euler = None
        # while self.zero_euler is None:
        #     try:
        #         zero_quat = np.array(self.imu.quaternion)
        #         self.zero_euler = R.from_quat(zero_quat).as_euler("xyz")
        #     except Exception as e:
        #         print(e)
        #         continue
        self.last_imu_data = [0, 0, 0, 0]
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def imu_worker(self):
        while True:
            s = time.time()
            try:
                raw_orientation = np.array(self.imu.quaternion)  # quat
                euler = R.from_quat(raw_orientation).as_euler("xyz")
            except Exception as e:
                print(e)
                continue

            # Converting to correct axes
            # euler = euler - self.zero_euler
            euler = [np.pi - euler[1], euler[2], -euler[0]]
            euler[1] += np.deg2rad(self.pitch_bias)

            final_orientation_quat = R.from_euler("xyz", euler).as_quat()

            self.imu_queue.put(final_orientation_quat)
            took = time.time() - s
            time.sleep(max(0, 1 / self.sampling_freq - took))

    def get_data(self, euler=False):
        try:
            self.last_imu_data = self.imu_queue.get(False)  # non blocking
        except Exception:
            pass

        if not euler:
            return self.last_imu_data
        else:
            return R.from_quat(self.last_imu_data).as_euler("xyz")
