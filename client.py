import argparse
import io
import queue
import socket
import struct
import threading
import time
import sys
import tkinter as tk
from dataclasses import dataclass, field

from PIL import Image, ImageTk

CAMERA_OPCODE = 1
WEIGHT_OPCODE = 2
MPU_OPCODE = 3
READY_OPCODE = 0
COMMAND_OPCODE = 3
ERROR_OPCODE = 6
UDP_HEADER_SIZE = 9
FRAME_TIMEOUT_SECONDS = 2.0
PACKAGE_WEIGHT_THRESHOLD = 10.0 #The difference in grams to determine if a package has been added or removed

#holds the data for a camera frame from the server
@dataclass
class FrameBuffer:
	total_chunks: int
	received: dict[int, bytes] = field(default_factory=dict)
	last_update: float = field(default_factory=time.monotonic)


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="IoT client camera viewer")
	parser.add_argument(
		"--host",
		type=str,
		default="127.0.0.1",
		help="Server host or IP (default: localhost)",
	)
	parser.add_argument(
		"--port",
		type=int,
		default=5110,
		help="Server TCP AND UDP port (default: 5110)",
	)
	parser.add_argument(
		"--package-delta-threshold",
		type=float,
		default=PACKAGE_WEIGHT_THRESHOLD,
		help="Weight delta in grams used to detect package changes (default: 10.0)",
	)
	return parser.parse_args()


def encode_message(message: str, opcode: int) -> bytes:
	return bytes([opcode]) + message.encode("utf-8")

#registers the client with the server and returns the port that we are working with
def open_client_session(server_host: str, server_port: int) -> tuple[socket.socket, int]:
	tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	tcp_socket.connect((server_host, server_port))
	tcp_socket.sendall(encode_message("ready", READY_OPCODE))
	return tcp_socket, tcp_socket.getsockname()[1]

#encodes and sends a message to the server
def send_command(tcp_socket: socket.socket, message: str, timeout: float = 2.0) -> str:
	payload = encode_message(message, COMMAND_OPCODE)
	tcp_socket.sendall(payload)
	tcp_socket.settimeout(timeout)
	try:
		response = tcp_socket.recv(4096)
	except socket.timeout:
		response = b""

	if not response:
		return "No response received."
	return response.decode("utf-8", errors="replace")

#does a few things, binds the socket to some port and then recieves data
#then decodes the data received based off of opcode
def receive_server_data(
	udp_port: int,
	frame_queue: queue.Queue[bytes],
	stop_event: threading.Event,
	package_delta_threshold: float,
) -> None:
	udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	udp_socket.bind(("0.0.0.0", udp_port))
	udp_socket.settimeout(0.5)

	buffers: dict[int, FrameBuffer] = {}
	last_weight_value: float | None = None
	package_present: bool | None = None
	try:
		while not stop_event.is_set():
			try:
				data, _address = udp_socket.recvfrom(2048)
			except socket.timeout:
				data = b""
			if data:
				opcode = data[0]

				if opcode == WEIGHT_OPCODE:
					if len(data) >= struct.calcsize("!Bdd"):
						_, ts, weight = struct.unpack("!Bdd", data)
						if last_weight_value is not None:
							delta = weight - last_weight_value
							if delta >= package_delta_threshold:
								package_present = True
								print("Package status: ADDED")
							elif delta <= -package_delta_threshold:
								package_present = False
								print("Package status: REMOVED")
						last_weight_value = weight
					else:
						last_weight_value = weight
					continue

				if opcode == MPU_OPCODE:
					if len(data) >= struct.calcsize("!BddddddddB"):
						_, _ts, _ax, _ay, _az, _gx, _gy, _gz, _temp, state_code = struct.unpack("!BddddddddB", data)
						print("Door closed" if state_code == 0 else "Door opened")
					elif len(data) >= struct.calcsize("!Bdddddddd"):
						print("Door state unknown")
					continue

				if opcode == ERROR_OPCODE:
					message = data[1:].decode("utf-8", errors="replace") if len(data) > 1 else "Unknown error"
					print(f"SERVER ALERT: {message}")
					continue

				if opcode != CAMERA_OPCODE:
					continue

				if len(data) < UDP_HEADER_SIZE:
					continue

				_, frame_id, chunk_idx, chunk_total = struct.unpack(
					"!BIHH", data[:UDP_HEADER_SIZE]
				)

				payload = data[UDP_HEADER_SIZE:]
				buffer = buffers.get(frame_id)
				if buffer is None:
					buffer = FrameBuffer(total_chunks=chunk_total)
					buffers[frame_id] = buffer

				if chunk_idx not in buffer.received:
					buffer.received[chunk_idx] = payload
				buffer.last_update = time.monotonic()

				if len(buffer.received) == buffer.total_chunks:
					frame_bytes = b"".join(buffer.received[idx] for idx in range(chunk_total))
					try:
						frame_queue.put_nowait(frame_bytes)
					except queue.Full:
						pass
					del buffers[frame_id]

			now = time.monotonic()
			stale_frames = [
				fid for fid, buf in buffers.items() if now - buf.last_update > FRAME_TIMEOUT_SECONDS
			]
			for fid in stale_frames:
				del buffers[fid]
	finally:
		udp_socket.close()

def validate_command(command: str) -> str:
	command = command.strip()
	if not command:
		print("Empty command, please enter a valid command.")
	if command is not None and len(command) > 1024:
		print("Command too long, please limit to 1024 characters.")
	match command:
		case "exit":
			return command
		case "lock":
			return command
		case "unlock":
			return command
		case "stop":
			return command
	valid_commands = {"exit", "lock", "stop", "unlock"}
	print(f"please enter a valid command from {valid_commands}")
	return ""

#this does not need to be its own function, but it is ran a few times elsewhere
def close_tcp_connection(tcp_socket: socket.socket) -> None:
	try:
		tcp_socket.shutdown(socket.SHUT_RDWR)
	except OSError:
		pass
	tcp_socket.close()


def terminal_command_loop(
	stop_event: threading.Event,
	tcp_socket: socket.socket,
	root: tk.Tk,
) -> None:
	while not stop_event.is_set():
		try:
			command = input("Enter command to send (or 'exit' to quit): ").strip()
		except EOFError:
			stop_event.set()
			break
		command = validate_command(command)
		if not command:
			continue
		if command.lower() == "exit":
			stop_event.set()
			close_tcp_connection(tcp_socket)
			root.after(0, root.quit)
			break

		try:
			response = send_command(tcp_socket, command)
			print(response)
		except OSError as exc:
			print(f"TCP error: {exc}", file=sys.stderr)

#retrieves frames from the queue and displays them in the GUI
def poll_frames(
	root: tk.Tk,
	frame_queue: queue.Queue[bytes],
	image_label: tk.Label,
) -> None:
	latest_frame = None
	try:
		while True:
			latest_frame = frame_queue.get_nowait()
	except queue.Empty:
		pass

	if latest_frame:
		try:
			image = Image.open(io.BytesIO(latest_frame))
			photo = ImageTk.PhotoImage(image)
			image_label.configure(image=photo)
			image_label.image = photo
		except Exception:
			pass

	root.after(30, poll_frames, root, frame_queue, image_label)


def on_close(
	stop_event: threading.Event,
	tcp_socket: socket.socket,
	root: tk.Tk,
) -> None:
	stop_event.set()
	close_tcp_connection(tcp_socket)
	root.destroy()


def check_shutdown(stop_event: threading.Event, root: tk.Tk) -> None:
	if stop_event.is_set():
		root.destroy()
		return
	root.after(100, check_shutdown, stop_event, root)

def main() -> None:
	args = parse_args()
	frame_queue: queue.Queue[bytes] = queue.Queue(maxsize=2)
	stop_event = threading.Event()
	tcp_socket, client_port = open_client_session(args.host, args.port)
	print(f"Registered with server on local port {client_port}")
	
	data_thread = threading.Thread(
		target=receive_server_data,
		args=(client_port, frame_queue, stop_event, args.package_delta_threshold),
		daemon=True,
	)
	data_thread.start()

	root = tk.Tk()
	root.title("IoT Client Camera Feed")

	image_label = tk.Label(root)
	image_label.pack(fill=tk.BOTH, expand=True)
	command_thread = threading.Thread(
		target=terminal_command_loop,
		args=(stop_event, tcp_socket, root),
		daemon=True,
	)
	command_thread.start()
	root.protocol("WM_DELETE_WINDOW", lambda: on_close(stop_event, tcp_socket, root))
	poll_frames(root, frame_queue, image_label)
	check_shutdown(stop_event, root)
	root.mainloop()


if __name__ == "__main__":
	main()
