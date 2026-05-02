import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

#Pins: First 2 are the orange wires, last 2 are the purple wires
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)

GPIO.output(17, GPIO.HIGH)
GPIO.output(27, GPIO.HIGH)
time.sleep(6)
GPIO.output(17, GPIO.LOw)
GPIO.output(27, GPIO.LOW)
time.sleep(6)
GPIO.output(22, GPIO.HIGH)
GPIO.output(23, GPIO.HIGH)
time.sleep(6)
GPIO.output(22, GPIO.LOw)
GPIO.output(23, GPIO.LOW)
print("Finished Test")

GPIO.cleanup()