import multiprocessing
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
import argparse

CAMERA_OPCODE = 1
WEIGHT_OPCODE = 2
MPU_OPCODE = 3
ACTUATOR_OPCODE = 5
ERROR_OPCODE = 6
UDP_HEADER_SIZE = 9
FRAME_TIMEOUT_SECONDS = 2.0
PACKAGE_WEIGHT_THRESHOLD = 10

@dataclass
class FrameBuffer:
    total_chunks: int
    received: dict[int, bytes] = field(default_factory=dict)
    last_update: float = field(default_factory=time.monotonic)


def process_camera_packet(
    udpsocket: socket.socket,
    data: bytes,
    clients,
    buffers: dict[int, FrameBuffer],
) -> None:
    if len(data) < UDP_HEADER_SIZE:
        return

    _, frame_id, chunk_idx, chunk_total = struct.unpack("!BIHH", data[:UDP_HEADER_SIZE])
    #send all camera data to every registered client
    #in the future, it would be good to do some sort of filtering to only send it to relevant clients
    for client in list(clients):
        udpsocket.sendto(data, tuple(client))

    payload = data[UDP_HEADER_SIZE:]
    buffer = buffers.get(frame_id)
    if buffer is None:
        buffer = FrameBuffer(total_chunks=chunk_total)
        buffers[frame_id] = buffer

    if chunk_idx not in buffer.received:
        buffer.received[chunk_idx] = payload
    buffer.last_update = time.monotonic()

    if len(buffer.received) == buffer.total_chunks:
        print(f"Received complete frame {frame_id} from camera")
        del buffers[frame_id]


def process_weight_packet(data: bytes) -> tuple[float, float] | None:
    if len(data) < struct.calcsize("!Bdd"):
        return None

    _, timestamp, weight = struct.unpack("!Bdd", data)
    print(f"Weight packet: t={timestamp:.3f}, weight={weight:.3f}")
    return (timestamp, weight)


def process_mpu_packet(
    data: bytes,
) -> tuple[float, tuple[float, float, float], tuple[float, float, float], float, str] | None:
    packet_with_state_size = struct.calcsize("!BddddddddB")
    packet_without_state_size = struct.calcsize("!Bdddddddd")
    if len(data) < packet_without_state_size:
        return None

    if len(data) >= packet_with_state_size:
        (
            _,
            timestamp,
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            temp_c,
            state_code,
        ) = struct.unpack("!BddddddddB", data[:packet_with_state_size])
        state = "CLOSED" if state_code == 0 else "OPEN"
    else:
        (
            _,
            timestamp,
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            temp_c,
        ) = struct.unpack("!Bdddddddd", data[:packet_without_state_size])
        state = "UNKNOWN"

    print(
        f"MPU packet: t={timestamp:.3f}, "
        f"accel=({ax:.3f},{ay:.3f},{az:.3f}), "
        f"gyro=({gx:.3f},{gy:.3f},{gz:.3f}), temp={temp_c:.2f}, state={state}"
    )
    return (timestamp, (ax, ay, az), (gx, gy, gz), temp_c, state)


def forward_packet_to_clients(udpsocket: socket.socket, data: bytes, clients) -> None:
    for client in list(clients):
        udpsocket.sendto(data, tuple(client))


def send_error_to_clients(udpsocket: socket.socket, clients, message: str) -> None:
    packet = bytes([ERROR_OPCODE]) + message.encode("utf-8")
    for client in list(clients):
        udpsocket.sendto(packet, tuple(client))


def has_numeric_deviation(previous: float | None, current: float, threshold: float) -> bool:
    if previous is None:
        return True
    print(f"Comparing previous={previous:.3f} to current={current:.3f} with threshold={threshold:.3f}")
    return abs(current - previous) >= threshold


def has_vector_deviation(
    previous: tuple[float, float, float] | None,
    current: tuple[float, float, float],
    threshold: float,
) -> bool:
    if previous is None:
        return True

    distance = ((current[0] - previous[0]) ** 2 + (current[1] - previous[1]) ** 2 + (current[2] - previous[2]) ** 2) ** 0.5
    return distance >= threshold


def start_actuator_action(action: str, actuator_target) -> None:
    if not actuator_target:
        raise RuntimeError("No actuator target registered by the rpi")

    packet = bytes([ACTUATOR_OPCODE]) + action.encode("utf-8")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        udp_socket.sendto(packet, tuple(actuator_target))
    print(f"Sent actuator command '{action}' to {tuple(actuator_target)}")


def create_shared_state(manager: multiprocessing.Manager):
    clients = manager.list()
    actuator_target = manager.dict()
    door_state = manager.dict()
    door_state["is_door_locked"] = False
    return clients, actuator_target, door_state


def start_udp_process(
    hostname: str,
    port: int,
    clients,
    actuator_target,
    door_state,
    weight_deviation_threshold: float,
) -> multiprocessing.Process:
    udp_process = multiprocessing.Process(
        target=data_server,
        args=(
            hostname,
            port,
            clients,
            actuator_target,
            door_state,
            weight_deviation_threshold,
        ),
        daemon=True,
    )
    udp_process.start()
    return udp_process


def start_tcp_thread(hostname: str, port: int, clients, actuator_target, door_state) -> threading.Thread:
    tcp_thread = threading.Thread(
        target=tcp_server,
        args=(hostname, port, clients, actuator_target, door_state),
        daemon=True,
    )
    tcp_thread.start()
    return tcp_thread

#receives data from the rpi
def data_server(
    hostname: str,
    port: int,
    clients,
    actuator_target,
    door_state,
    weight_deviation_threshold: float,
):
    udpsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udpsocket.bind((hostname, port))
    buffers: dict[int, FrameBuffer] = {}
    last_weight: tuple[float, float] | None = None
    last_mpu_state: str | None = None
    weight_violation_active = False
    door_open_violation_active = False
    while True:
        data, address = udpsocket.recvfrom(2048)
        if not data:
            continue

        opcode = data[0]

        if opcode == CAMERA_OPCODE:
            process_camera_packet(udpsocket=udpsocket, data=data, clients=clients, buffers=buffers)

            now = time.monotonic()
            stale_frames = [
                fid
                for fid, buf in buffers.items()
                if now - buf.last_update > FRAME_TIMEOUT_SECONDS
            ]
            for fid in stale_frames:
                del buffers[fid]
            continue

        if opcode == WEIGHT_OPCODE:
            parsed_weight = process_weight_packet(data)
            if parsed_weight is not None:
                _, weight = parsed_weight
                weight_changed = has_numeric_deviation(
                    last_weight[1] if last_weight is not None else None,
                    weight,
                    weight_deviation_threshold,
                )
                if weight_changed:
                    forward_packet_to_clients(udpsocket, data, clients)

                if door_state.get("is_door_locked", False) and weight_changed:
                    if not weight_violation_active:
                        send_error_to_clients(
                            udpsocket,
                            clients,
                            "ERROR: Weight changed while door is locked",
                        )
                        weight_violation_active = True
                else:
                    weight_violation_active = False
                last_weight = parsed_weight
            continue

        if opcode == MPU_OPCODE:
            parsed_mpu = process_mpu_packet(data)
            if parsed_mpu is not None:
                _, _, _, _, state = parsed_mpu
                if state != last_mpu_state:
                    forward_packet_to_clients(udpsocket, data, clients)

                if door_state.get("is_door_locked", False) and state == "OPEN":
                    if not door_open_violation_active:
                        send_error_to_clients(
                            udpsocket,
                            clients,
                            "ERROR: Door opened while locked",
                        )
                        door_open_violation_active = True
                else:
                    door_open_violation_active = False

                last_mpu_state = state
            continue

        print(f"Ignoring unknown UDP opcode {opcode} from {address}")


def handle_tcp_client(conn, address, clients, actuator_target, door_state):
    client_address = conn.getpeername()
    try:
        print("TCP connection from {address}!".format(address=address))
        while True:
            data = conn.recv(1024)
            if not data:
                break

            opcode = data[0:1]
            if opcode == b'\x00':
                command = data[1:].decode("utf-8")
                print("Received command: {command}".format(command=command))
                if command == "ready":
                    if client_address not in clients:
                        print(f"Adding {client_address} to clients")
                        clients.append(client_address)
                elif command.startswith("ready actuator "):
                    actuator_port = int(command.split()[-1])
                    actuator_target["address"] = (client_address[0], actuator_port)
                    print(f"Registered actuator target at {(client_address[0], actuator_port)}")
            elif opcode == b'\x03':
                command = data[1:].decode("utf-8")
                print("Received command: {command}".format(command=command))
                if command == "extend":
                    start_actuator_action("extend", actuator_target.get("address"))
                    conn.sendall(b"OK actuator extending")
                elif command == "retract":
                    start_actuator_action("retract", actuator_target.get("address"))
                    conn.sendall(b"OK actuator retracting")
                elif command == "stop":
                    start_actuator_action("stop", actuator_target.get("address"))
                    conn.sendall(b"OK actuator stopped")
                elif command == "close":
                    door_state["is_door_locked"] = True
                    conn.sendall(b"OK door locked")
                elif command == "unlock":
                    door_state["is_door_locked"] = False
                    conn.sendall(b"OK door unlocked")
                else:
                    conn.sendall(b"OK")
    finally:
        if client_address in clients:
            clients.remove(client_address)
        conn.close()


def tcp_server(hostname, port, clients, actuator_target, door_state):
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_socket.bind((hostname, port))
    tcp_socket.listen(5)
    while True:
        conn, address = tcp_socket.accept()
        thread = threading.Thread(
            target=handle_tcp_client,
            args=(conn, address, clients, actuator_target, door_state),
            daemon=True,
        )
        thread.start()


def main() -> None:
    args = parse_args()
    hostname = args.host
    print("Starting server on {hostname}, with port {port}".format(hostname=hostname, port=args.port))
    manager = multiprocessing.Manager()
    clients, actuator_target, door_state = create_shared_state(manager)
    start_udp_process(
        hostname,
        args.port,
        clients,
        actuator_target,
        door_state,
        PACKAGE_WEIGHT_THRESHOLD,
    )
    start_tcp_thread(hostname, args.port, clients, actuator_target, door_state)
    while True:
        time.sleep(1)

def parse_args() -> argparse.Namespace:
    DEFAULT_HOST="0.0.0.0"
    DEFAULT_PORT=5110
    parser = argparse.ArgumentParser(description="Capture Raspberry Pi camera feed")
    parser.add_argument(
		"--host",
		type=str,
		default=DEFAULT_HOST,
		help="put different ip than 0.0.0.0 as needed",
	)
    parser.add_argument(
		"--port",
		type=int,
		default=DEFAULT_PORT,
		help="port for all udp and tcp connections (default: 5110)",
	)
    return parser.parse_args()

    
if __name__ == "__main__":
    main()