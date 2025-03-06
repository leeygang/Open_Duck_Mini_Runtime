# from mini_bdx_runtime.hwi_feetech_pypot import HWI
from mini_bdx_runtime.rustypot_position_hwi import HWI
import time

hwi = HWI()
hwi.turn_on()
time.sleep(1)
