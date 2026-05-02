# Smart Mailbox

A brief one-line description of your IOT project.

Code for a more secure mailbox, with package weight and door sensing, camera feed alerts, and remote lock and unlock to prevent package theft.  

## Table of Contents

- [Overview](#overview)  

- [Hardware Components](#hardware-components)  

- [Software and Dependencies](#software-and-dependencies)  

- [Usage](#usage)  

- [Results and Demonstration](#results-and-demonstration)  


## Overview

Describe the objective of your project, the problem it solves, and the main features.

The problem we hope to solve is package theft. Through sensors keeping track of packages, the door, and the outside with a camera, we can increase the security and safety of delivered mail and packages. Our project, the Smart Mailbox, in its current state, alerts a client or user of added and removed packages in real time, the opening or closing of the door, and live camera feed for demo purposes. We also have remote locking and unlocking from a user device. The underlying mechanism enabling this is socket programing, primarily UDP but also a TCP connection.   

## Hardware Components

Amount	| Hardware Components
--------+------------------------------------
1		| HX711 5kg load cell
1		| Load Cell Scale Kit (1)
1		| Arducam RPi camera module
1		| MPU 6050 Gyroscope & Accelerometer
1		| DC 5V-50mm-15N 15mm s Electric Linear Actuator Motor
1		| Raspberry Pi 3 B+ model
2		| Breadboards

(1) You can buy one but for us we built one with 2 15mm screws, 2 washers, and wood.

## Software and Dependencies

We used Python 3.10.12 for our project code. 
Libraries for the Raspberry Pi and the seperate Server/Client Code is listed in the corresponding requirements.txt file

# Server/Client Code
pillow==10.4.0
python-dotenv==1.2.2
requests==2.33.1

*You might be missing tkinter, so if thats the case, follow this command in you linux terminal* 
bash$ sudo apt-get update 
bash$ sudo apt-get install python3-tk 

# Raspberry Pi Code
picamera==1.13
mpu6050-raspberrypi
git+https://github.com/gandalf15/HX711.git#egg=HX711&subdirectory=HX711_Python3
numpy==2.4.4

*You will likely have to pip3 install these in a virtual environment*
*If you do, make sure to use --system-site-packages when creating it, or just edit venv/pyvenv.cfg*

## Usage

# Raspberry Pi 3 & Sensor Wiring
HX711 5kg load cell:
VCC = PIN 18 			(3V)
SCK = clock = PIN 18	(GPIO 5)
DT = data out = PIN 22 	(GPIO 6)
GND = PIN 6 			(GND, any other ground pin also works)

Arducam RPi camera module:
Attach ribbon cable to the Raspberry Pi's Camera Module Port
Guide if needed: https://projects.raspberrypi.org/en/projects/getting-started-with-picamera/1

MPU 6050 Gyroscope & Accelerometer:
VCC = PIN 2 			(5V)
GND = PIN 39			(GND, any other ground pin also works)
SCL = PIN 5 			(SCL)
SDA = PIN 3 			(SDA)

DC 5V-50mm-15N 15mm s Electric Linear Actuator Motor:
Use GPIO pins and connect to an H-bridge circuit
Vcc = PIN 4								(5V)
GND = PIN 9 							(GND)
Orange Wires (Top left and bottom right transistor gates) 
Top Left Transistor Gate = PIN 11 		(GPIO 0)
Bottom Right Transistor Gate = PIN 13	(GPIO 2) 
Purple Wires (Top left and bottom right transistor gates) 
Top Right Transistor Gate = PIN 15 		(GPIO 3)
Bottom Left Transistor Gate = PIN 16	(GPIO 4) 

Motor V+ pin = node between left side transistors
Motor V- pin = node between left side transistors
Guide if needed: https://www.build-electronic-circuits.com/h-bridge/

# Running the Code
On a seperate device, i.e. laptop or computer, run the server.py code. 
bash$ python3 server.py --host {hostname} --port {port number}

Then run the Client.py code
bash$ python3 client.py --host {hostname} --port {port number}

Then, on the Raspberry Pi, run the rpi.py code
bash$ python3 rpi.py --host {hostname} --port {port number} --camera --weight --accel --actuator

*Where the {...}'s are optional python command line arguments.*
*By default, everything uses port 5110*
*The client and RPi should use the server's IP for their hostname arguments.*
*The servers hostname is usually optional unless youre doing weird stuff with your ip, ie your hostname is seperate from your actual ip on the device, everything else should be set*

To from the client terminal, you can type extend, stop, and retract to control the Linear Actuator Motor.
Typing exit will close the client.

## Results and Demonstration

Acheived remote lock and unlocking. 
Could keep track of the opening amd closing of the door.
Weight sensor would detect and alert the user of added and removed packages. 
Was able to view the camera feed from the user's device.  
