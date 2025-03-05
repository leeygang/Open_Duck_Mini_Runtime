import RPi.GPIO as GPIO #Importe la bibliothèque pour contrôler les GPIOs
import numpy as np
import time

GPIO.setmode(GPIO.BCM)  # Use physical pin numbering
# GPIO.setmode(GPIO.BOARD) #Définit le mode de numérotation (Board)
GPIO.setwarnings(False) #On désactive les messages d'alerte

LED = 23 #Définit le numéro du port GPIO qui alimente la led

GPIO.setup(LED, GPIO.OUT) #Active le contrôle du GPIO

# state = GPIO.input(LED) #Lit l'état actuel du GPIO, vrai si allumé, faux si éteint

GPIO.output(LED, GPIO.HIGH) #On l'allume

clignottement_time = 0.1

cli_s = time.time()
low = False
while True:
    if not low and np.random.rand() > 0.99:
        print("cli")
        GPIO.output(LED, GPIO.LOW)
        cli_s = time.time()
        low = True

    if time.time() - cli_s > clignottement_time:
        GPIO.output(LED, GPIO.HIGH)
        low = False

    time.sleep(0.01)




# if state : #Si GPIO allumé
#     GPIO.output(LED, GPIO.LOW) #On l’éteint
# else : #Sinon
#     GPIO.output(LED, GPIO.HIGH) #On l'allume
