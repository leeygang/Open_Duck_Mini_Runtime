import socket
import time
import numpy as np
import pickle
from queue import Queue
from threading import Thread
from scipy.spatial.transform import Rotation as R
from FramesViewer.viewer import Viewer
import argparse


class IMUClient:
    def __init__(self, host, port=1234, freq=30):
        self.host = host
        self.port = port
        self.freq = freq
        self.client_socket = socket.socket()
        self.client_socket.connect((self.host, self.port))
        self.imu_queue = Queue(maxsize=1)
        self.last_imu = [0, 0, 0, 0]

        Thread(target=self.imu_worker, daemon=True).start()

    def imu_worker(self):
        while True:
            try:
                data = self.client_socket.recv(1024)  # receive response
                data = pickle.loads(data)

                self.imu_queue.put(data)
            except:
                print("missed imu")

            time.sleep(1 / self.freq)

    def get_imu(self):
        try:
            self.last_imu = self.imu_queue.get(False)
        except:
            pass

        return self.last_imu


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pitch_bias", type=float, default=0, help="deg")
    args = parser.parse_args()

    client = IMUClient("192.168.166.253")
    fv = Viewer()
    fv.start()
    pose = np.eye(4)
    pose[:3, 3] = [0.1, 0.1, 0.1]

    while True:
        quat = client.get_imu()
        try:
            euler = R.from_quat(quat).as_euler("xyz")
            euler[1] += np.deg2rad(args.pitch_bias)
            rot_mat = R.from_euler("xyz", euler).as_matrix()
            # rot_mat = R.from_quat(quat).as_matrix()
            # print(euler)
            pose[:3, :3] = rot_mat
            fv.pushFrame(pose, "aze")
        except:
            pass
        time.sleep(1 / 30)
