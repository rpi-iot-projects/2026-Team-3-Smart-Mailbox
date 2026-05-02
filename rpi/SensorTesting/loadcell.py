import RPi.GPIO as GPIO
import time
from hx711 import HX711

GPIO.setmode(GPIO.BCM)

hx = HX711(dout_pin=25, pd_sck_pin=24)

while True:
	reading = hx.get_raw_data_mean()
	print("Reading:")
	print(reading)
	time.sleep(0.1)
	