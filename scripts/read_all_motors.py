from pypot.feetech import FeetechSTS3215IO
import time
io = FeetechSTS3215IO(
    "/dev/ttyACM0",
    baudrate=1000000,
    use_sync_read=True,
)

ids = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33]

io.disable_torque(ids)
while True:
    present_positions = io.get_present_position(ids)
    print(present_positions)
    print("--")
    time.sleep(0.1)
