import socket
import time
import pickle
from mini_bdx_runtime.imu import Imu

host = "0.0.0.0"
port = 1234

server_socket = socket.socket()
server_socket.bind((host, port))

imu = Imu(50, pitch_bias=20)

while True:
    server_socket.listen(1)
    conn, address = server_socket.accept()  # accept new connection
    print("Connection from: " + str(address))
    try:
        while True:
            data = imu.get_data()
            data = pickle.dumps(data)
            conn.send(data)  # send data to the client
            time.sleep(1 / 30)
    except:
        pass
