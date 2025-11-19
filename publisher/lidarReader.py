# lidar_reader.py

import threading
import time
from typing import List, Tuple

from rplidar import RPLidar
from hardwareLidar import create_lidar

# Data type for a single LIDAR point: (angle in radians, distance in mm)
LidarPoint = Tuple[float, float]

class LidarReader:
    def __init__(self, port: str | None = None, reconnect_delay: float = 1.0):
        """
        port: if None, use create_lidar() to auto-detect
        reconnect_delay: time to wait (seconds) before retrying connection on error
        """
        self.port = port
        self.reconnect_delay = reconnect_delay

        self._lidar: RPLidar | None = None
        self._thread: threading.Thread | None = None
        self._running = False

        self._lock = threading.Lock()
        self._latest_points: List[LidarPoint] = []

    # ====== API public ======

    def start(self):
        """Start the thread to read LIDAR data."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the thread and close the LIDAR."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._close_lidar()

    def get_latest_points(self) -> List[LidarPoint]:
        """Get the latest list of points (angle_rad, distance_mm)."""
        with self._lock:
            return list(self._latest_points)

    # ====== Internal ======

    def _loop(self):
        """Main loop: connect + read data + auto reconnect."""
        while self._running:
            try:
                if self._lidar is None:
                    print("[LidarReader] Connecting to LIDAR...")
                    self._lidar = create_lidar(self.port)
                    # clear cache
                    self._lidar.clear_input()

                print("[LidarReader] Start scanning")
                for scan in self._lidar.iter_scans():
                    if not self._running:
                        break

                    points: List[LidarPoint] = []
                    for (_, angle_deg, dist_mm) in scan:
                        angle_rad = angle_deg * 3.14159265 / 180.0
                        points.append((angle_rad, dist_mm))

                    with self._lock:
                        self._latest_points = points

            except Exception as e:
                print(f"[LidarReader] Error: {e}")
                self._close_lidar()
                if not self._running:
                    break
                print(f"[LidarReader] Reconnecting in {self.reconnect_delay} s...")
                time.sleep(self.reconnect_delay)

        print("[LidarReader] Thread stopped")

    def _close_lidar(self):
        """Closing lidar connection."""
        try:
            if self._lidar is not None:
                print("[LidarReader] Closing LIDAR")
                try:
                    self._lidar.stop()
                except Exception:
                    pass
                try:
                    self._lidar.disconnect()
                except Exception:
                    pass
        finally:
            self._lidar = None
