# subscriber.py
import json
import threading
import paho.mqtt.client as mqtt
from collections import deque

broker = "localhost"
port = 1883
topic = "lidar/scan"


# Ring buffer
class LidarBuffer:
    def __init__(self, max_packets=3):
        self.buffer = deque(maxlen=max_packets)
        self.lock = threading.Lock()

    def push(self, points):
        with self.lock:
            self.buffer.append(points)

    def get_latest(self):
        with self.lock:
            if len(self.buffer) > 0:
                return self.buffer[-1]
            return None


# Global buffer để plotter import
lidar_buffer = LidarBuffer()


#Json decompressing function
def decompress_points(comp):
    pts = []
    for angle_u16, dist_u16 in comp:
        angle = angle_u16 / 10000.0     # rad
        dist = float(dist_u16)
        pts.append((angle, dist))
    return pts


# MQTT message callback
def on_message(client, userdata, msg):
    try:
        packet = json.loads(msg.payload.decode("utf-8"))
        pts = decompress_points(packet["p"])
        lidar_buffer.push(pts)
    except Exception as e:
        print("[Subscriber] error:", e)


# Start subscriber in background thread
def start_subscriber_background():
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(broker, port, 60)
    client.subscribe(topic, qos=0)

    # chạy không block GUI
    thread = threading.Thread(target=client.loop_forever, daemon=True)
    thread.start()

    print("[Subscriber] Running in background thread...")
