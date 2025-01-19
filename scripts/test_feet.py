import RPi.GPIO as GPIO

LEFT_FOOT_PIN = 22
RIGHT_FOOT_PIN = 27
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BCM) # Use physical pin numbering
GPIO.setup(LEFT_FOOT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RIGHT_FOOT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True: # Run forever
    print("=")
    if GPIO.input(LEFT_FOOT_PIN) == GPIO.LOW:
        print("LEFT!")
    if GPIO.input(RIGHT_FOOT_PIN) == GPIO.LOW:
        print("RIGHT! ")
