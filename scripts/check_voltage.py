from pypot.feetech import FeetechSTS3215IO

io = FeetechSTS3215IO(
    "/dev/ttyACM0",
    baudrate=1000000,
    use_sync_read=True,
)

id = 20

print(io.get_present_voltage([id]))