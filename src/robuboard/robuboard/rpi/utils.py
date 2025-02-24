from pathlib import Path
import re
CPUINFO_PATH = Path("/proc/cpuinfo")

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
    return is_raspberry_pi() and is_mmteensy()

if __name__ == '__main__':
    print("Is Raspberry Pi: ", is_raspberry_pi())
    print("Is Raspberry Pi-CM: ", is_raspberry_pi_cm())
    print("Is MMTeensy: ", is_mmteensy())
    print("Is ROBU-Baord: ", is_robuboard())

