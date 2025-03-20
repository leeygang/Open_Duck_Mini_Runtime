from pypot.feetech import FeetechSTS3215IO
import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument(
    "--port",
    help="The port the motor is connected to. Default is /dev/ttyACM0",
    default="/dev/ttyACM0",
)
parser.add_argument("--id", help="The id of the motor.", type=str, required=True)
args = parser.parse_args()


current_id = int(args.id)
io = FeetechSTS3215IO("/dev/ttyACM0")
current_max_acceleration = io.get_maximum_acceleration([current_id])
current_acceleration = io.get_acceleration([current_id])

print(f"current max acceleration: {current_max_acceleration}")
print(f"current acceleration: {current_acceleration}")


io.set_lock({current_id: 0})

io.set_maximum_acceleration({current_id: 0})
io.set_acceleration({current_id: 0})
print("done setting accel settings")
time.sleep(1)
