import subprocess
import sys
import importlib
from pathlib import Path
import os

# Required packages
REQUIRED_PACKAGES = [
    "pyqt5",
    "matplotlib",
    "rplidar",
    "paho-mqtt"
]

def check_and_install_package(pkg_name: str, import_name: str | None = None):
    """Check and install the package if it's not already installed."""
    if import_name is None:
        import_name = pkg_name

    try:
        importlib.import_module(import_name)
        print(f"[install.lib] {import_name} is already installed.")
    except ImportError:
        print(f"[install.lib] Installing {pkg_name}...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", pkg_name])
        print(f"[install.lib] Installed {pkg_name}.")

def is_venv_active():
    """Check if the virtual environment is active."""
    return hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix)

def create_venv():
    """Create a virtual environment (venv) if it doesn't exist."""
    print("[install.lib] Creating virtual environment (venv)...")
    venv_path = Path("venv")
    
    if not venv_path.exists():
        # Create virtual environment
        subprocess.check_call([sys.executable, "-m", "venv", "venv"])
        print("[install.lib] Virtual environment (venv) created.")
    else:
        print("[install.lib] Virtual environment (venv) already exists.")

    print("[install.lib] To activate the virtual environment, use the following command:")
    print("On Windows: venv\\Scripts\\activate.bat")
    print("On Linux/Mac: source venv/bin/activate")

def activate_venv():
    """Activate the virtual environment manually."""
    if not is_venv_active():
        print("[install.lib] WARNING: Virtual environment is not activated!")
        print("[install.lib] Please activate it by running:")
        print("On Windows: venv\\Scripts\\activate.bat")
        print("On Linux/Mac: source venv/bin/activate")
        sys.exit(1)  # Exit the script to prompt user to activate venv
    else:
        print("[install.lib] Virtual environment is active.")

def install_required_packages():
    """Install all required packages."""
    for pkg in REQUIRED_PACKAGES:
        check_and_install_package(pkg)

def install_mosquitto_linux():
    """Install Mosquitto for Linux."""
    print("[install.lib] Installing Mosquitto on Linux...")
    subprocess.check_call(["sudo", "apt-get", "install", "-y", "mosquitto", "mosquitto-clients"])
    subprocess.check_call(["sudo", "systemctl", "enable", "mosquitto"])
    subprocess.check_call(["sudo", "systemctl", "start", "mosquitto"])
    print("[install.lib] Mosquitto installation successful!")

def install_mosquitto_windows():
    """Guide for installing Mosquitto on Windows."""
    print("[install.lib] To install Mosquitto on Windows, please download it from: https://mosquitto.org/download/")

def setup_mosquitto():
    """Install Mosquitto depending on the OS."""
    if sys.platform == "linux" or sys.platform == "linux2":
        install_mosquitto_linux()
    elif sys.platform == "win32":
        install_mosquitto_windows()

def setup():
    """Set up the environment and required libraries."""
    print("[install.lib] Starting environment setup...")

    # Create and activate venv
    create_venv()

    # Check if venv is active
    activate_venv()

    # Install required packages
    install_required_packages()

    # Install Mosquitto
    setup_mosquitto()

    print("[install.lib] Everything has been successfully set up.")

if __name__ == "__main__":
    setup()
