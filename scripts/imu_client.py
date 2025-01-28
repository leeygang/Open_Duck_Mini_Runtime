import socket
import time
import pickle
from queue import Queue
from threading import Thread


class IMUClient:
    def __init__(self, host, port=1234, freq=10):
        self.host = host
        self.port = port
        self.freq = 30
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
    client = IMUClient("192.168.89.246")

    while True:
        print(client.get_imu())
        time.sleep(1 / 30)
