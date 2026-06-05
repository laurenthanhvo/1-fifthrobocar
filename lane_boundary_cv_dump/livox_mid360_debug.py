import socket
import struct
import time
import numpy as np

LIDAR_IP = "192.168.1.199"
LISTEN_PORT = 56301

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", LISTEN_PORT))
sock.settimeout(2.0)

print(f"Listening for Livox point packets on UDP :{LISTEN_PORT}")

count = 0
last_print = time.time()
points_total = 0

while True:
    data, addr = sock.recvfrom(2048)

    if addr[0] != LIDAR_IP:
        continue

    if len(data) < 36:
        continue

    version = data[0]
    pkt_len = struct.unpack_from("<H", data, 1)[0]
    dot_num = struct.unpack_from("<H", data, 5)[0]
    data_type = data[10]

    points = []

    # Data type 1: Cartesian int32 x/y/z in mm, reflectivity uint8, tag uint8
    if data_type == 1:
        offset = 36
        stride = 14

        for i in range(dot_num):
            base = offset + i * stride
            if base + stride > len(data):
                break

            x_mm, y_mm, z_mm = struct.unpack_from("<iii", data, base)
            refl = data[base + 12]
            points.append((x_mm, y_mm, z_mm, refl))

    # Data type 2: Cartesian int16 x/y/z in 10 mm units
    elif data_type == 2:
        offset = 36
        stride = 8

        for i in range(dot_num):
            base = offset + i * stride
            if base + stride > len(data):
                break

            x_raw, y_raw, z_raw = struct.unpack_from("<hhh", data, base)
            refl = data[base + 6]
            points.append((x_raw * 10, y_raw * 10, z_raw * 10, refl))

    else:
        continue

    if not points:
        continue

    arr = np.array(points, dtype=np.float32)
    count += 1
    points_total += len(points)

    now = time.time()
    if now - last_print > 1.0:
        x = arr[:, 0] / 1000.0
        y = arr[:, 1] / 1000.0
        z = arr[:, 2] / 1000.0

        print(
            f"packets={count} pts/sec~={points_total} "
            f"type={data_type} dot_num={dot_num} len={len(data)} "
            f"x=[{x.min():.2f},{x.max():.2f}]m "
            f"y=[{y.min():.2f},{y.max():.2f}]m "
            f"z=[{z.min():.2f},{z.max():.2f}]m"
        )

        points_total = 0
        last_print = now
