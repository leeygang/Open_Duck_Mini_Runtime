import adafruit_bno055
import numpy as np
import serial
from queue import Queue
from threading import Thread
import time
from scipy.spatial.transform import Rotation as R


class Imu:
    def __init__(self, sampling_freq, pitch_bias=0):
        self.sampling_freq = sampling_freq
        self.pitch_bias = pitch_bias

        self.uart = serial.Serial("/dev/ttyS0")  # , baudrate=115200)
        self.imu = adafruit_bno055.BNO055_UART(self.uart)
        self.imu.mode = adafruit_bno055.IMUPLUS_MODE
        self.last_imu_data = [0, 0, 0, 0]
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def imu_worker(self):
        while True:
            try:
                s = time.time()
                raw_orientation = self.imu.quaternion  # quat
                print("getting orientation took", time.time() - s)
                s = time.time()
                euler = R.from_quat(raw_orientation).as_euler("xyz")
            except Exception as e:
                print(e)
                continue

            # Converting to correct axes
            euler = [np.pi - euler[1], euler[2], -euler[0]]
            euler[1] += np.deg2rad(self.pitch_bias)

            final_orientation_quat = R.from_euler("xyz", euler).as_quat()

            self.imu_queue.put(final_orientation_quat)
            print("all other stuff took", time.time() - s)
            time.sleep(1 / (self.sampling_freq/2))

    def get_data(self, euler=False):
        try:
            self.last_imu_data = self.imu_queue.get(False)  # non blocking
        except Exception:
            pass

        if not euler:
            return self.last_imu_data
        else:
            return R.from_quat(self.last_imu_data).as_euler("xyz")
