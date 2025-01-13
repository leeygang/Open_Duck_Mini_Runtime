from pypot.feetech import FeetechSTS3215IO
# from mini_bdx_runtime.hwi_feetech_pypot import HWI
import time


io = FeetechSTS3215IO(
    "/dev/ttyACM0",
    baudrate=1000000,
    use_sync_read=True,
)
id = 24

io.enable_torque([id])
io.set_mode({id: 0})
io.set_goal_position({id: 0})
time.sleep(1)
exit()

hwi = HWI()
# hwi.turn_off()
hwi.turn_on()
