import RPi.GPIO as GPIO
import time

class ServoController:
    def __init__(self, pin1, pin2):
        self.pin1 = pin1
        self.pin2 = pin2

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)

        # Create PWM objects for both servos (50Hz frequency)
        self.pwm1 = GPIO.PWM(self.pin1, 50)
        self.pwm2 = GPIO.PWM(self.pin2, 50)

        self.pwm1.start(0)
        self.pwm2.start(0)

    def map_input_to_angle(self, value):
        """
        Maps an input range of [-1, 1] to an angle range of [0°, 180°].
        """
        return 90 + (value * 90)  # Maps -1 to 0°, 0 to 90°, and 1 to 180°

    def set_position(self, servo, value):
        """
        Moves the servo based on an input value in the range [-1, 1].
        :param servo: 1 for the first servo, 2 for the second servo
        :param value: A float between -1 and 1
        """
        if -1 <= value <= 1:
            angle = self.map_input_to_angle(value)
            duty = 2 + (angle / 18)  # Convert angle to duty cycle (1ms-2ms)
            if servo == 1:
                self.pwm1.ChangeDutyCycle(duty)
            elif servo == 2:
                self.pwm2.ChangeDutyCycle(duty)
            else:
                print("Invalid servo number!")
            # time.sleep(0.3)  # Allow time for movement
        else:
            print("Invalid input! Enter a value between -1 and 1.")

    def stop(self):
        """Stops PWM and cleans up GPIO."""
        self.pwm1.stop()
        self.pwm2.stop()
        GPIO.cleanup()

# Initialize the ServoController for GPIO12 and GPIO13
servo_control = ServoController(pin1=12, pin2=13)

try:
    while True:
        servo_num = int(input("Enter servo number (1 or 2): "))
        value = float(input("Enter position (-1 to 1): "))
        servo_control.set_position(servo_num, value)

except KeyboardInterrupt:
    print("Stopping servos...")
    servo_control.stop()
