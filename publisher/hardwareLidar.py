#For Linux and Windows

import sys
from rplidar import RPLidar

def detect_default_port():
    """
    Choose default port candidates based on OS.:
    - Windows: try COM3, COM4...
    - Linux (Pi): use /dev/ttyUSB0, /dev/ttyUSB1...
    You can adjust this list based on your actual setup.
    """
    if sys.platform.startswith("win"):
        candidate_ports = ["COM7", "COM11"]
    else:
        candidate_ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyAMA0"]

    return candidate_ports

def create_lidar(port=None):
    """
    Returns an RPLidar instance connected to the specified port or
    tries to auto-detect the port if none is provided.
    :param port: Specific port to connect to (e.g., 'COM3' or '/dev/ttyUSB0')
    :return: RPLidar instance
    """
    if port:
        # Use specific port
        return RPLidar(port)

    last_error = None
    for p in detect_default_port():
        try:
            print(f"[hardware_lidar] Trying port: {p}")
            lidar = RPLidar(p)
            print(f"[hardware_lidar] Connected to LIDAR on {p}")
            return lidar
        except Exception as e:
            print(f"[hardware_lidar] Failed on {p}: {e}")
            last_error = e

    raise RuntimeError(f"Could not connect to LIDAR on any default port. Last error: {last_error}")
