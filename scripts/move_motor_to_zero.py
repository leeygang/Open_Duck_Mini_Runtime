import argparse

from pypot.feetech import FeetechSTS3215IO
import time

parser = argparse.ArgumentParser()
parser.add_argument("--id", type=int)
args = parser.parse_args()

io = FeetechSTS3215IO(
    "/dev/ttyACM0",
    baudrate=1000000,
    use_sync_read=True,
)


print(io.get_present_position([args.id])[0])

exit()
