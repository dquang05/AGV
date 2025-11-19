import json
import time
import paho.mqtt.client as mqtt
from lidarReader import LidarReader

broker = "localhost"
port = 1883
topic = "lidar/scan"

SEND_INTERVAL = 0.1  # 100ms


def compress_points(points):
    """Convert float (angle_rad, dist_mm) -> uint16 compressed form."""
    compressed = []

    for angle, dist in points:
        angle_u16 = int(angle * 10000)      # rad → uint16
        angle_u16 = max(0, min(angle_u16, 65535))

        dist_u16 = int(dist)                # mm → uint16
        dist_u16 = max(0, min(dist_u16, 65535))

        compressed.append([angle_u16, dist_u16])

    return compressed


def main():
    reader = LidarReader(port=None, reconnect_delay=1.0)
    reader.start()

    client = mqtt.Client()
    client.connect(broker, port, 60)
    client.loop_start()

    print("[Publisher] Sending 100ms compressed JSON packets")

    packet_id = 0

    try:
        while True:
            points = reader.get_latest_points()
            if points:
                comp = compress_points(points)

                packet = {
                    "id": packet_id,
                    "t": int(time.time() * 1000),
                    "p": comp
                }
                packet_id += 1

                json_data = json.dumps(packet, separators=(',', ':'))
                client.publish(topic, json_data)

                print(f"[Publisher] Sent packet {packet['id']} ({len(comp)} pts)")

            time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        print("[Publisher] Stopped by user")

    reader.stop()
    client.loop_stop()


if __name__ == "__main__":
    main()
