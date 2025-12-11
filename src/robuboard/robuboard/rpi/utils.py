from pathlib import Path
import re
CPUINFO_PATH = Path("/proc/cpuinfo")

from smbus2 import SMBus, i2c_msg
from threading import Thread
import time

global IS_ROBUBOARD_V1
global IS_ROBUBOARD

IS_ROBUBOARD_V1:bool = False
IS_ROBUBOARD = False

def _ping_worker(bus, address, result):
    """
    Wird im Thread ausgeführt. Versucht ein I2C-Write-Quick.
    Ergebnis wird in result[0] gespeichert.
    """
    try:
        with SMBus(bus) as bus:
            msg = i2c_msg.write(address, [])
            bus.i2c_rdwr(msg)
        result.append(True)
    except OSError:
        result.append(False)
    except Exception:
        result.append(False)


def i2c_ping(bus: int, address: int, timeout: float = 0.1) -> bool:
    """
    Pingt eine I2C-Adresse mit Timeout.
    Rückgabe:
      True  → Gerät antwortet
      False → Kein Gerät oder Timeout
    """
    result = []
    thread = Thread(target=_ping_worker, args=(bus, address, result))
    thread.start()
    thread.join(timeout)

    if thread.is_alive():
        # Thread hängt → Timeout erreicht
        return False

    return result[0] if result else False


def is_raspberry_pi():
    if not CPUINFO_PATH.exists():
        return False
    with open(CPUINFO_PATH) as f:
        cpuinfo = f.read()
    return re.search(r"^Model\s*:\s*Raspberry Pi", cpuinfo, flags=re.M) is not None

def is_raspberry_pi_cm():
    if not CPUINFO_PATH.exists():
        return False
    with open(CPUINFO_PATH) as f:
        cpuinfo = f.read()
    return re.search(r"^Model\s*:\s*Raspberry Pi Compute Module", cpuinfo, flags=re.M) is not None


def is_bootloader_teensy():
    import subprocess
    try:
        result = subprocess.run(["lsusb"], capture_output=True, text=True, check=True)
        return "Teensy Halfkay" in result.stdout
    except subprocess.CalledProcessError:
        return False 

def is_serial_teensy():
    import subprocess
    try:
        result = subprocess.run(["lsusb"], capture_output=True, text=True, check=True)
        return "Teensyduino Serial" in result.stdout
    except subprocess.CalledProcessError:
        return False 

    
def is_mmteensy():
    import subprocess
    try:
        result = subprocess.run(["lsusb"], capture_output=True, text=True, check=True)
        return "Teensy" in result.stdout
    except subprocess.CalledProcessError:
        return False 
    
# def is_mmteensy():
#     from glob import glob
#     try:

#         teensy_detected = True if len(glob("/dev/serial/by-id/*Teensy*")) == 1 else False
#     except Exception:
#         teensy_detected = False
#     return teensy_detected

def is_robuboard():
    global IS_ROBUBOARD_V1
    global IS_ROBUBOARD

    IS_ROBUBOARD = is_raspberry_pi() and is_mmteensy()
    IS_ROBUBOARD_V1 = False
    if IS_ROBUBOARD: IS_ROBUBOARD_V1 = i2c_ping(0, 0x41)
    return IS_ROBUBOARD

def is_robuboard_v1():
    global IS_ROBUBOARD_V1
    is_robuboard()
    return IS_ROBUBOARD_V1

is_robuboard()

if __name__ == '__main__':
    print("Is Raspberry Pi: ", is_raspberry_pi())
    print("Is Raspberry Pi-CM: ", is_raspberry_pi_cm())
    print("Is MMTeensy: ", is_mmteensy())
    print("Is ROBU-Baord: ", is_robuboard())

