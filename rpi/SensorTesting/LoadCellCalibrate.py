import RPi.GPIO as GPIO
import time
from hx711 import HX711

GPIO.setmode(GPIO.BCM)
hx = HX711(dout_pin=25, pd_sck_pin=24)

hx.zero()

input('Place known weight on scale, press enter: ')
reading = hx.get_raw_data_mean(readings=150)

knownWeight = input('Weight of known object in grams: ') #in grams
value = float(knownWeight)
ratio = reading/value
print(ratio)
print("====================")
hx.set_scale_ratio(ratio)

while True:
	weight = hx.get_weight_mean()
	print(weight)
	time.sleep(0.1)
	