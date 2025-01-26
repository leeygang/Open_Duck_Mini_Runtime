import board
import busio
import time
import adafruit_bno055
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = adafruit_bno055.IMUPLUS_MODE

while True:
    s = time.time()
    quat = sensor.quaternion
    print(quat)
    print("getting quat took : ", round(time.time()-s, 3))
    time.sleep(0.01)
