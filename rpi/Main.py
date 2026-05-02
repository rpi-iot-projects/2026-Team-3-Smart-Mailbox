import numpy as np
import mpu6050
from hx711 import HX711
import RPi.GPIO as GPIO
import time

def getAvgAccel(mpu, samples=20):
    vals = []
    for _ in range(samples):
        data = mpu.get_accel_data()
        vals.append([data['x'], data['y'], data['z']])
        time.sleep(0.02)
    return np.mean(vals, axis=0)

def angleBetween(v1, v2):
    v1 = np.array(v1)
    v2 = np.array(v2)
    
    #Normalize
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    
    #Dot product to angle
    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
    angle = np.degrees(np.arccos(dot))
    
    return angle

mpu6050 = mpu6050.mpu6050(0x68)
closedVector = getAvgAccel(mpu6050)

CLOSED_THRESHOLD = 10   #Need to tune
LOAD_CELL_RATIO = 9.5   #Need to test more?

GPIO.setmode(GPIO.BCM)
hx = HX711(dout_pin=25,pd_sck_pin=24)
hx.set_scale_ratio(LOAD_CELL_RATIO)
previousWeight = NULL

while True:
    #Read Accelerometer For Door State
    accel = mpu6050.get_accel_data()
    currentVector = [accel['x'], accel['y'], accel['z']]    
    angle = angleBetween(closedVector, currentVector)

    if angle < CLOSED_THRESHOLD:
        state = "CLOSED"
    else:
        state = "OPEN"

    print(f"Angle: {angle:.2f} degrees, State: {state}")
    
    #Check if Phone wants to lock/unlock the door.
    #NEED TO DO
    
    weight = hx.get_weight_mean(readings=70)
    print(weight)
    
    #TODO
    #logic for change in weight -> if theres a big enough difference, send SMS & other stuff.
    
    time.sleep(0.1)
