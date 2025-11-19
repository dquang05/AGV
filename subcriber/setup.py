import subprocess
import sys
import importlib
from pathlib import Path

# Required packages for subscriber (no need for rplidar)
REQUIRED_PACKAGES = [
    "paho-mqtt",
    "matplotlib", 
    "pyqt5",
]

def check_and_install_package(pkg_name: str, import_name: str | None = None):
    """Check and install the package if it's not already installed."""
    if import_name is None:
        import_name = pkg_name

    try:
        importlib.import_module(import_name)
        print(f"[setup.py] {import_name} is already installed.")
    except ImportError:
        print(f"[setup.py] Installing {pkg_name}...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", pkg_name])
        print(f"[setup.py] Installed {pkg_name}.")

def is_venv_active():
    """Check if the virtual environment is active."""
    return hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix)

def activate_venv():
    """Require user to activate venv if it's not active."""
    if not is_venv_active():
        print("[setup.py] WARNING: Virtual environment is not activated!")
        print("[setup.py] Please activate the virtual environment first.")
        print("On Windows: venv\\Scripts\\activate.bat")
        print("On Linux/Mac: source venv/bin/activate")
        sys.exit(1)  # Exit the script to prompt user to activate venv
    else:
        print("[setup.py] Virtual environment is active.")

def install_required_packages():
    """Install all required packages."""
    for pkg in REQUIRED_PACKAGES:
        check_and_install_package(pkg)

def setup():
    """Set up the environment and required libraries."""
    print("[setup.py] Starting environment setup...")

    # Create and activate venv
    print("[setup.py] Checking if venv is activated...")
    activate_venv()

    # Install required packages
    print("[setup.py] Installing required packages...")
    install_required_packages()

    print("[setup.py] Everything has been successfully set up.")

if __name__ == "__main__":
    setup()
