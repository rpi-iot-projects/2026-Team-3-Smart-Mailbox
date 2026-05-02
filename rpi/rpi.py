from __future__ import annotations

import argparse
import io
import socket
import struct
import time
from pathlib import Path
import threading
import numpy as np
import mpu6050
import RPi.GPIO as GPIO
from hx711 import HX711
from picamera2 import Picamera2

CAMERA_OPCODE = 1
WEIGHT_OPCODE = 2
MPU_OPCODE = 3
MPU6050_DEFAULT_ADDR = 0x68

HOST = "127.0.0.1" #the rpi sends data to this ip
PORT = 5110 #the rpi sends data to this port, note that the tcp listener port is seperate, see ACTUATOR_COMMAND_PORT
#Below are many magic numbers, these have all been adjusted and calibrated for after testing
HX711_DOUT_PIN = 25 
HX711_PD_SCK_PIN = 24
HX711_SCALE_RATIO = -428 
HX711_READINGS = 70
SCALE_INTERVAL = 0.1
ACTUATOR_IN1_PIN = 17
ACTUATOR_IN2_PIN = 27
MPU_BUS_ID = 1
MPU_INTERVAL = 0.05
MPU_RECONNECT_INTERVAL = 1.0
CLOSED_THRESHOLD_DEGREES = 6.0
MOVEMENT_START_THRESHOLD_DEGREES = 15.0
FORCE_OPEN_THRESHOLD_DEGREES = 45.0
FORCE_CLOSE_THRESHOLD_DEGREES = 2.0
SETTLE_WINDOW_SECONDS = 2.0
SIGNIFICANT_ANGLE_STEP_DEGREES = 1.5
CAMERA_FPS = 10
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_PAYLOAD_SIZE = 1200
ACTUATOR_COMMAND_PORT = 5111 #command input port
#This setup for the accelerometer needs to be done globally 
#Doing it in a function can cause issues if the accelerometer was not enabled at some point in the past
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)

def actuator_stop() -> None:
	GPIO.output(22, GPIO.LOW)
	GPIO.output(23, GPIO.LOW)
	GPIO.output(17, GPIO.LOW)
	GPIO.output(27, GPIO.LOW)

def actuator_extend() -> None:
	GPIO.output(17, GPIO.HIGH)
	GPIO.output(27, GPIO.HIGH)

def actuator_retract() -> None:
	GPIO.output(22, GPIO.HIGH)
	GPIO.output(23, GPIO.HIGH)


def register_actuator_listener(server_host: str, server_port: int, actuator_port: int) -> None:
	print(f"Registering actuator listener with server at {server_host}:{server_port}, actuator command port {actuator_port}")
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp_socket:
		tcp_socket.connect((server_host, server_port))
		tcp_socket.sendall(bytes([0]) + f"ready actuator {actuator_port}".encode("utf-8"))


def actuator_command_loop(
	listen_host: str = "0.0.0.0",
	listen_port: int = ACTUATOR_COMMAND_PORT,
	in1_pin: int = ACTUATOR_IN1_PIN,
	in2_pin: int = ACTUATOR_IN2_PIN,
) -> None:
	command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	command_socket.bind((listen_host, listen_port))
	print(f"Actuator command loop listening on {listen_host}:{listen_port}")
	try:
		while True:
			data, address = command_socket.recvfrom(1024)
			if not data:
				continue

			if len(data) < 2:
				continue

			opcode = data[0]
			action = data[1:].decode("utf-8", errors="ignore").strip().lower()
			print(f"Actuator packet from {address}: opcode={opcode}, action={action}")

			if opcode != 5:
				continue

			if action == "extend":
				actuator_extend()
			elif action == "retract":
				actuator_retract()
			elif action == "stop":
				actuator_stop()
	finally:
		actuator_stop()
		command_socket.close()


def get_avg_accel(
	mpu_sensor,
	samples: int = 20,
	sleep_seconds: float = 0.02,
) -> np.ndarray:
	vals: list[list[float]] = []
	for _ in range(samples):
		accel = mpu_sensor.get_accel_data()
		vals.append([accel["x"], accel["y"], accel["z"]])
		time.sleep(max(0.0, sleep_seconds))
	return np.mean(vals, axis=0)


def angle_between(v1, v2) -> float:
	v1 = np.array(v1, dtype=float)
	v2 = np.array(v2, dtype=float)
	v1_norm = np.linalg.norm(v1)
	v2_norm = np.linalg.norm(v2)
	if v1_norm == 0.0 or v2_norm == 0.0:
		return 0.0

	v1 = v1 / v1_norm
	v2 = v2 / v2_norm
	dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
	return float(np.degrees(np.arccos(dot)))


def ensure_mpu_connected(
	address: int,
	reconnect_interval: float,
) -> tuple[mpu6050.mpu6050, np.ndarray]:
	while True:
		try:
			sensor = mpu6050.mpu6050(address)
			baseline = get_avg_accel(sensor)
			print("MPU connected; continuing sensor reads")
			return sensor, baseline
		except Exception as exc:
			print(f"MPU not available ({exc}); retrying in {reconnect_interval:.1f}s")
			time.sleep(max(0.1, reconnect_interval))


def read_accel(
	bus_id: int = 1,
	address: int = MPU6050_DEFAULT_ADDR,
	interval: float = 0.05,
	udp_host: str | None = None,
	udp_port: int = 5110,
	on_value=None,
) -> None:
	udp_socket: socket.socket | None = None
	remote_addr: tuple[str, int] | None = None
	if udp_host:
		udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		remote_addr = (udp_host, udp_port)

	mpu6050a: mpu6050.mpu6050 | None = None
	closed_vector: np.ndarray | None = None
	stable_state = "CLOSED"
	in_motion = False
	last_angle: float | None = None
	last_significant_motion_ts = time.time()

	print("Starting MPU read loop")
	while True:
		if mpu6050a is None or closed_vector is None:
			mpu6050a, closed_vector = ensure_mpu_connected(address, MPU_RECONNECT_INTERVAL)
			stable_state = "CLOSED"
			in_motion = False
			last_angle = None
			last_significant_motion_ts = time.time()
			print("Initial resting state set to CLOSED")

		try:
			accel = mpu6050a.get_accel_data()
			gyro = mpu6050a.get_gyro_data()
			try:
				temp_c = float(mpu6050a.get_temp())
			except Exception:
				temp_c = 0.0
		except Exception as exc:
			print(f"MPU read failed ({exc}); waiting for reconnect")
			mpu6050a = None
			closed_vector = None
			time.sleep(max(0.1, MPU_RECONNECT_INTERVAL))
			continue

		ax = float(accel["x"])
		ay = float(accel["y"])
		az = float(accel["z"])
		gx = float(gyro["x"])
		gy = float(gyro["y"])
		gz = float(gyro["z"])
		timestamp = time.time()

		current_vector = (ax, ay, az)
		angle = angle_between(closed_vector, current_vector)
		candidate_state = "CLOSED" if angle < CLOSED_THRESHOLD_DEGREES else "OPEN"

		#state changes for very large/small angles so major turns are
		#reflected immediately even if settle timers are extended by sensor noise.
		if stable_state == "CLOSED" and angle >= FORCE_OPEN_THRESHOLD_DEGREES:
			stable_state = "OPEN"
			in_motion = False
			last_significant_motion_ts = timestamp
			print(f"Forced state transition at angle={angle:.2f} -> OPEN")
		elif stable_state == "OPEN" and angle <= FORCE_CLOSE_THRESHOLD_DEGREES:
			stable_state = "CLOSED"
			in_motion = False
			last_significant_motion_ts = timestamp
			print(f"Forced state transition at angle={angle:.2f} -> CLOSED")

		if last_angle is None:
			last_angle = angle

		angle_delta = abs(angle - last_angle)
		if angle_delta >= SIGNIFICANT_ANGLE_STEP_DEGREES:
			last_significant_motion_ts = timestamp

		if not in_motion and angle >= MOVEMENT_START_THRESHOLD_DEGREES:
			in_motion = True
			last_significant_motion_ts = timestamp
			print(f"Movement detected at angle={angle:.2f}")

		if in_motion:
			settled = (timestamp - last_significant_motion_ts) >= SETTLE_WINDOW_SECONDS
			if settled:
				if candidate_state != stable_state:
					stable_state = candidate_state
					print(f"State transition settled at angle={angle:.2f} -> {stable_state}")

				in_motion = False

		if on_value is None:
			print(
				f"{timestamp:.3f} angle={angle:.2f} state={stable_state} "
				f"accel=({ax:.3f},{ay:.3f},{az:.3f}) "
				f"gyro=({gx:.3f},{gy:.3f},{gz:.3f}) temp={temp_c:.2f}"
			)
		else:
			on_value(timestamp, (ax, ay, az), (gx, gy, gz), temp_c)

		if udp_socket is not None and remote_addr is not None:
			send_mpu_over_udp(
				udp_socket,
				remote_addr,
				timestamp,
				ax,
				ay,
				az,
				gx,
				gy,
				gz,
				temp_c,
				stable_state,
			)

		last_angle = angle
		time.sleep(max(0.0, interval))

#packet format: uint8 opcode, float64 timestamp, 6x float64 (3 for accel, 3 for gyro), float64 temp, uint8 state
def send_mpu_over_udp(
	udp_socket: socket.socket,
	remote_addr: tuple[str, int],
	timestamp: float,
	ax: float,
	ay: float,
	az: float,
	gx: float,
	gy: float,
	gz: float,
	temp_c: float,
	state: str,
) -> None:
	state_code = 0 if state == "CLOSED" else 1
	packet = struct.pack("!BddddddddB", MPU_OPCODE, timestamp, ax, ay, az, gx, gy, gz, temp_c, state_code)
	udp_socket.sendto(packet, remote_addr)

def read_scale_data(
	dout_pin: int = HX711_DOUT_PIN,
	pd_sck_pin: int = HX711_PD_SCK_PIN,
	scale_ratio: float = HX711_SCALE_RATIO,
	readings: int = HX711_READINGS,
	interval: float = SCALE_INTERVAL,
	udp_host: str | None = None,
	udp_port: int = 5110,
	on_value=None,
) -> None:
	udp_socket: socket.socket | None = None
	remote_addr: tuple[str, int] | None = None
	if udp_host:
		udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		remote_addr = (udp_host, udp_port)

	GPIO.setmode(GPIO.BCM)
	hx = HX711(dout_pin=dout_pin, pd_sck_pin=pd_sck_pin)
	hx.zero()
	hx.set_scale_ratio(scale_ratio)

	while True:
		value = hx.get_weight_mean(readings=readings)
		if value is None:
			time.sleep(max(0.0, interval))
			continue

		timestamp = time.time()
		if on_value is None:
			print(f"{timestamp:.3f} weight={value}")
		else:
			on_value(timestamp, value)

		if udp_socket is not None and remote_addr is not None:
			send_weight_over_udp(udp_socket, remote_addr, timestamp, value)

		time.sleep(max(0.0, interval))

#packet format: uint8 opcode, float64 timestamp, float64 weight.
def send_weight_over_udp(
	udp_socket: socket.socket,
	remote_addr: tuple[str, int],
	timestamp: float,
	weight: float,
) -> None:
	
	packet = struct.pack("!Bdd", WEIGHT_OPCODE, timestamp, weight)
	udp_socket.sendto(packet, remote_addr)

#capture camera input forever
#returns a tuple of (frames_captured, udp_packets_sent)
def gather_camera_feed(
	fps: int,
	width: int,
	height: int,
	udp_host: str,
	udp_port: int,
	udp_payload_size: int,
) -> tuple[int, int]:

	udp_socket: socket.socket
	remote_addr: tuple[str, int]
	udp_packets_sent = 0
	udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	remote_addr = (udp_host, udp_port)

	frame_count = 0
	camera = Picamera2()
	config = camera.create_video_configuration(main={"size": (width, height)})
	camera.configure(config)
	camera.start()
	time.sleep(2)

	frame_interval = 1.0 / max(fps, 1)
	while True:
		capture_started = time.time()
		frame_stream = io.BytesIO()
		camera.capture_file(frame_stream, format="jpeg")
		frame_bytes = frame_stream.getvalue()

		if udp_socket is not None and remote_addr is not None:
			udp_packets_sent += send_frame_over_udp(
				udp_socket=udp_socket,
				remote_addr=remote_addr,
				frame_id=frame_count,
				frame_bytes=frame_bytes,
				payload_size=udp_payload_size,
			)

		frame_count += 1
		elapsed = time.time() - capture_started
		if elapsed < frame_interval:
			time.sleep(frame_interval - elapsed)
	camera.stop()
	udp_socket.close()

	return frame_count, udp_packets_sent


#packet format: uint8 opcode, uint32 frame_id, uint16 chunk_idx, uint16 chunk_total.
def send_frame_over_udp(
	udp_socket: socket.socket,
	remote_addr: tuple[str, int],
	frame_id: int,
	frame_bytes: bytes,
	payload_size: int,
) -> int:

	chunk_total = (len(frame_bytes) + payload_size - 1) // payload_size
	if chunk_total == 0:
		return 0

	if chunk_total > 65535:
		raise ValueError(
			"Frame requires too many UDP chunks; reduce resolution/quality or increase payload_size"
		)

	packets_sent = 0
	for chunk_idx in range(chunk_total):
		start = chunk_idx * payload_size
		end = start + payload_size
		chunk = frame_bytes[start:end]
		header = struct.pack("!BIHH", CAMERA_OPCODE, frame_id, chunk_idx, chunk_total)
		udp_socket.sendto(header + chunk, remote_addr)
		packets_sent += 1
		print(f"Sent UDP packet for frame {frame_id}, chunk {chunk_idx}/{chunk_total}, to {remote_addr}")
	return packets_sent


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser()
	parser.add_argument(
		"--host",
		type=str,
		default=HOST,
		help="Remote IP to send UDP packets to",
	)
	parser.add_argument(
		"--port",
		type=int,
		default=PORT,
		help="UDP AND TCP port of the server (default: 5110)",
	)
	parser.add_argument(
		"--camera",
		action="store_true",
		help="Enables the camera(default: disabled)",
	)
	parser.add_argument(
		"--weight",
		action="store_true",
		help="Enable weight sensor (default: disabled)",
	)
	parser.add_argument(
		"--accel",
		action="store_true",
		help="Enable MPU sensor (default: disabled)",
	)
	parser.add_argument(
		"--actuator",
		action="store_true",
		help="enable actuator and process commands",
	)
	return parser.parse_args()


if __name__ == "__main__":
	args = parse_args()
	udp_host = args.host
	udp_port = args.port
	threads: list[threading.Thread] = []
	#run each sensor in their own thread as needed
	if args.weight:
		scale_thread = threading.Thread(
			target=read_scale_data,
			kwargs={
				"dout_pin": HX711_DOUT_PIN,
				"pd_sck_pin": HX711_PD_SCK_PIN,
				"scale_ratio": HX711_SCALE_RATIO,
				"readings": HX711_READINGS,
				"interval": SCALE_INTERVAL,
				"udp_host": udp_host,
				"udp_port": udp_port,
			},
			daemon=False,
		)
		scale_thread.start()
		threads.append(scale_thread)

	if args.accel:
		mpu_thread = threading.Thread(
			target=read_accel,
			kwargs={
				"bus_id": MPU_BUS_ID,
				"address": MPU6050_DEFAULT_ADDR,
				"interval": MPU_INTERVAL,
				"udp_host": udp_host,
				"udp_port": udp_port,
			},
			daemon=False,
		)
		mpu_thread.start()
		threads.append(mpu_thread)

	if args.actuator:
		register_actuator_listener(udp_host, udp_port, ACTUATOR_COMMAND_PORT)
		actuator_thread = threading.Thread(
			target=actuator_command_loop,
			kwargs={
				"listen_host": "0.0.0.0",
				"listen_port": ACTUATOR_COMMAND_PORT,
				"in1_pin": ACTUATOR_IN1_PIN,
				"in2_pin": ACTUATOR_IN2_PIN,
			},
			daemon=False,
		)
		actuator_thread.start()
		threads.append(actuator_thread)

	if args.camera:
		frames, packets = gather_camera_feed(
			fps=CAMERA_FPS,
			width=CAMERA_WIDTH,
			height=CAMERA_HEIGHT,
			udp_host=udp_host,
			udp_port=udp_port,
			udp_payload_size=CAMERA_PAYLOAD_SIZE,
		)
	elif threads:
		for thread in threads:
			thread.join()
	else:
		print("No sensors enabled. Use --camera, --weight, --accel, and/or --actuator.")

