from pathlib import Path
import re
CPUINFO_PATH = Path("/proc/cpuinfo")

def is_raspberry_pi():
    if not CPUINFO_PATH.exists():
        return False
    with open(CPUINFO_PATH) as f:
        cpuinfo = f.read()
    return re.search(r"^Model\s*:\s*Raspberry Pi", cpuinfo, flags=re.M) is not None

if __name__ == '__main__':
    print("Is Raspberry Pi: ", is_raspberry_pi())

