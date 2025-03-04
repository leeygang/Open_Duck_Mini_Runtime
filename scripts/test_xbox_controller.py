from mini_bdx_runtime.xbox_controller import XBoxController
import time

xbox = XBoxController(50)
while True:
    print(xbox.get_last_command())
    time.sleep(0.1)