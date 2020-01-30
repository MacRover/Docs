import Jetson.GPIO as GPIO
import time


GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

pin = 11

GPIO.setup( pin, GPIO.OUT)


GPIO.output(pin,GPIO.LOW)

GPIO.output(pin,GPIO.HIGH)
time.sleep(0.5)
GPIO.output(pin,GPIO.LOW)
time.sleep(0.5)


#GPIO.cleanup(pin)
