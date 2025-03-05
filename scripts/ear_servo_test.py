import numpy as np

import RPi.GPIO as GPIO
import time
# from time import sleep

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
pwm = GPIO.PWM(12, 50)
pwm.start(0)  # Start the servo with 0 duty cycle ( at 0 deg position )

# pwm.ChangeDutyCycle(0)
# time.sleep(1)

# pwm.stop(0)  # Stop the servo with 0 duty cycle ( at 0 deg position )
# GPIO.cleanup()  # Clean up all the ports we've used.

start = time.time()
while True:
    if time.time() - start > 3:
        break
    target_pwm = 10.0*np.sin(2*np.pi*1*time.time()) + 10
    pwm.ChangeDutyCycle(target_pwm)
    time.sleep(0.02)

# # Set up the  PWM on pin #16 at 50Hz
# pwm.ChangeDutyCycle(5)  # Tells the servo to turn to the left ( -90 deg position )
# sleep(0.5)  # Tells the servo to Delay for 5sec
# pwm.ChangeDutyCycle(7.5)  # Tells the servo to turn to the neutral position ( at 0 deg position )
# sleep(0.5)  # Tells the servo to Delay for 5sec
# pwm.ChangeDutyCycle(10)  # Tells the servo to turn to the right ( +90 deg position )
# sleep(0.5)  # Tells the servo to Delay for 5sec
pwm.stop(0)  # Stop the servo with 0 duty cycle ( at 0 deg position )
GPIO.cleanup()  # Clean up all the ports we've used.
