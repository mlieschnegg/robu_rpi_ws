from robuboard.rpi import utils as board_utils
from robuboard.rpi.utils import (
    is_mmteensy,
    is_robuboard,
    is_bootloader_teensy,
)
from robuboard.rpi.utils import GPIOCHIP_HANLDE
from neopixel.common.neopixel_spi_write import neopixel_spi_write
import time
import sys
import subprocess
import os

from smbus2 import smbus2 as smbus
import spidev
import lgpio

GPIO_TEENSY_RESET = 23
GPIO_POWER_SWITCH = 25
GPIO_POWER_REGULATOR_EN = 26
GPIO_STATUS_LED = 21

PCA9536_ADDR = 0x41
PCA9536_REG_CONFIG = 0x03
PCA9536_REG_OUTPUT = 0x01

PCA9536_BIT_TEENSY_RESET = 0
PCA9536_BIT_TEENSY_BOOT = 1

PCAL6408A_ADDR = 0x20
PCAL6408A_REG_CONFIG = 0x03
PCAL6408A_REG_OUTPUT = 0x01
PCAL6408A_BIT_TEENSY_ONOFF = 3
PCAL6408A_BIT_TEENSY_BOOT = 5
PCAL6408A_TEENSY_OUTPUT_MASK = (
    (1 << PCAL6408A_BIT_TEENSY_ONOFF) | (1 << PCAL6408A_BIT_TEENSY_BOOT)
)
# P2/P4 monitor the active-low switch nets; never drive these pins.
PCAL6408A_TEENSY_INPUT_MASK = (1 << 2) | (1 << 4)

robuboard_init_gpios: bool = False
robuboard_enable_5v_supply_on: bool = False


def _gpio_write_once(pin: int, value: int, retries: int = 3, delay: float = 0.01):
    """
    Claim output temporarily, write value, then release.
    Retries if GPIO is busy.
    """
    last_error = None

    for attempt in range(retries):
        handle = lgpio.gpiochip_open(GPIOCHIP_HANLDE)
        try:
            lgpio.gpio_claim_output(handle, pin, value)
            lgpio.gpio_write(handle, pin, value)
            return  # success

        except lgpio.error as e:
            last_error = e
            if "GPIO busy" in str(e) and attempt < retries - 1:
                time.sleep(delay)
            else:
                raise RuntimeError(f"Failed to write GPIO {pin}: {e}")

        finally:
            try:
                lgpio.gpiochip_close(handle)
            except Exception:
                pass

    raise RuntimeError(f"Failed to write GPIO {pin} after {retries} retries: {last_error}")


def _gpio_read_once(pin: int, retries: int = 3, delay: float = 0.01) -> int:
    """
    Claim input temporarily, read value, then release.
    Retries if GPIO is busy.
    """
    last_error = None

    for attempt in range(retries):
        handle = lgpio.gpiochip_open(GPIOCHIP_HANLDE)
        try:
            lgpio.gpio_claim_input(handle, pin)
            return lgpio.gpio_read(handle, pin)

        except lgpio.error as e:
            last_error = e
            if "GPIO busy" in str(e) and attempt < retries - 1:
                time.sleep(delay)
            else:
                raise RuntimeError(f"Failed to read GPIO {pin}: {e}")

        finally:
            try:
                lgpio.gpiochip_close(handle)
            except Exception:
                pass

    raise RuntimeError(f"Failed to read GPIO {pin} after {retries} retries: {last_error}")

def _open_spi_for_led():
    candidates = [
        (1, 0, "/dev/spidev1.0"),  # CM5 typisch
        (0, 0, "/dev/spidev0.0"),  # CM4 typisch
    ]

    for bus, dev, path in candidates:
        if os.path.exists(path):
            try:
                spi = spidev.SpiDev()
                spi.open(bus, dev)
                return spi, path
            except Exception as e:
                print(f"SPI {path} exists but failed to open: {e}")

    raise RuntimeError("No usable SPI device found")

def init_gpios():
    global robuboard_init_gpios

    if not robuboard_init_gpios:
        is_robuboard()

        if board_utils.IS_ROBUBOARD_V3:
            with smbus.SMBus(1) as bus:
                # Preload inactive LOW levels before enabling the MOSFET gates.
                output = bus.read_byte_data(PCAL6408A_ADDR, PCAL6408A_REG_OUTPUT)
                bus.write_byte_data(
                    PCAL6408A_ADDR,
                    PCAL6408A_REG_OUTPUT,
                    output & ~PCAL6408A_TEENSY_OUTPUT_MASK,
                )
                config = bus.read_byte_data(PCAL6408A_ADDR, PCAL6408A_REG_CONFIG)
                bus.write_byte_data(
                    PCAL6408A_ADDR,
                    PCAL6408A_REG_CONFIG,
                    (config | PCAL6408A_TEENSY_INPUT_MASK)
                    & ~PCAL6408A_TEENSY_OUTPUT_MASK,
                )
        elif board_utils.IS_ROBUBOARD_V1:
            with smbus.SMBus(1) as bus:
                # Preload inactive LOW, then configure P0/P1 as outputs.
                bus.write_byte_data(PCA9536_ADDR, PCA9536_REG_OUTPUT, 0x00)
                bus.write_byte_data(PCA9536_ADDR, PCA9536_REG_CONFIG, 0x0C)
        else:
            # Teensy reset inactive default
            _gpio_write_once(GPIO_TEENSY_RESET, 0)

        # 5V default enable
        enable_5v_supply()

        robuboard_init_gpios = True


def _teensy_expander():
    if board_utils.IS_ROBUBOARD_V3:
        return (
            PCAL6408A_ADDR,
            PCAL6408A_REG_OUTPUT,
            PCAL6408A_BIT_TEENSY_ONOFF,
            PCAL6408A_BIT_TEENSY_BOOT,
        )
    if board_utils.IS_ROBUBOARD_V1:
        return (
            PCA9536_ADDR,
            PCA9536_REG_OUTPUT,
            PCA9536_BIT_TEENSY_RESET,
            PCA9536_BIT_TEENSY_BOOT,
        )
    return None


def _pulse_expander_pin(address: int, register: int, bit: int, duration: float):
    with smbus.SMBus(1) as bus:
        output = bus.read_byte_data(address, register)
        bus.write_byte_data(address, register, output | (1 << bit))
        try:
            time.sleep(duration)
        finally:
            # Keep unrelated outputs, including USB-C enables, unchanged.
            output = bus.read_byte_data(address, register)
            bus.write_byte_data(address, register, output & ~(1 << bit))


def _pulse_teensy_onoff(duration: float):
    expander = _teensy_expander()
    if expander is not None:
        address, register, onoff_bit, _ = expander
        _pulse_expander_pin(address, register, onoff_bit, duration)
    else:
        _gpio_write_once(GPIO_TEENSY_RESET, 1)
        try:
            time.sleep(duration)
        finally:
            _gpio_write_once(GPIO_TEENSY_RESET, 0)


def is_on_5v_supply() -> bool:
    return robuboard_enable_5v_supply_on


def enable_5v_supply():
    global robuboard_enable_5v_supply_on
    _gpio_write_once(GPIO_POWER_REGULATOR_EN, 1)
    robuboard_enable_5v_supply_on = True


def disable_5v_supply():
    global robuboard_enable_5v_supply_on
    _gpio_write_once(GPIO_POWER_REGULATOR_EN, 0)
    robuboard_enable_5v_supply_on = False


def get_power_switch() -> bool:
    return bool(_gpio_read_once(GPIO_POWER_SWITCH))


def power_off_robuboard():
    init_gpios()
    print(f"handle: {GPIOCHIP_HANLDE}")
    enable_5v_supply()
    print("Powering off RobuBoard ...")
    subprocess.run(["sync"])
    disable_5v_supply()
    time.sleep(5)
    print("Cannot power off RobuBoard!")
    # subprocess.run(["shutdown", "now"])


def power_off_teensy():
    init_gpios()

    enable_5v_supply()
    print("powering off teensy...")

    _pulse_teensy_onoff(5)


def power_on_teensy():
    init_gpios()

    power_off_teensy()
    print("powering on teensy...")

    _pulse_teensy_onoff(1.0)


def start_firmware_teensy(timeout_s: float = 5.0) -> bool:
    """
    Starts Teensy firmware via teensy_loader_cli.
    Returns True on success, False otherwise.
    """
    init_gpios()
    enable_5v_supply()
    print("Starting firmware on teensy ...")

    cmd = ["teensy_loader_cli", "--mcu=TEENSY_MICROMOD", "-s", "-b", "-v"]

    try:
        completed = subprocess.run(
            cmd,
            timeout=timeout_s,
            check=True,
            text=True,
            capture_output=True,
        )

        if completed.stdout:
            print(completed.stdout.strip())
        if completed.stderr:
            print(completed.stderr.strip())

        return True

    except subprocess.TimeoutExpired as e:
        print(f"ERROR: teensy_loader_cli timed out after {timeout_s:.1f}s.")
        if e.stdout:
            print("stdout:", e.stdout.strip())
        if e.stderr:
            print("stderr:", e.stderr.strip())
        return False

    except subprocess.CalledProcessError as e:
        print(f"ERROR: teensy_loader_cli failed with exit code {e.returncode}.")
        if e.stdout:
            print("stdout:", e.stdout.strip())
        if e.stderr:
            print("stderr:", e.stderr.strip())
        return False

    except FileNotFoundError:
        print("ERROR: teensy_loader_cli not found. Is it installed and in PATH?")
        return False


def start_bootloader_teensy(force=False):
    init_gpios()
    enable_5v_supply()

    expander = _teensy_expander()
    in_bootloader = is_bootloader_teensy()
    if in_bootloader and not force:
        print("bootloader allready activated!")
    elif expander is not None:
        print("starting bootloader on teensy...")
        address, register, _, boot_bit = expander
        _pulse_expander_pin(address, register, boot_bit, 0.1)
    elif is_mmteensy() and not in_bootloader:
        print("starting bootloader on teensy...")
        firmware_path: str = "/home/robu/work/robocup/robocup-teensy/.pio/build/teensymm/firmware.hex"
        subprocess.run(
            ["teensy_loader_cli", "--mcu=TEENSY_MICROMOD", "-s", firmware_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    elif in_bootloader:
        print("bootloader allready activated!")
    else:
        print("invalid state of teensy! Press boot switch!")


def upload_firmware_teensy(
    firmware_path: str = "/home/robu/work/robocup/robocup-teensy/.pio/build/teensymm/firmware.hex"
):
    init_gpios()
    ()
    print("uploading firmware to teensy...")
    subprocess.run(["teensy_loader_cli", "--mcu=TEENSY_MICROMOD", "-s", "-w", firmware_path])


def build_firmware_teensy(
    firmware_path: str = "/home/robu/work/robocup/robocup-teensy/"
):
    init_gpios()
    enable_5v_supply()
    print("building firmware for teensy...")
    subprocess.run(["pio", "run"], cwd=firmware_path)


def start_status_led_with_sudo(r: int = 255, g: int = 255, b: int = 51):
    command = f"sudo python3 -c 'from robuboard.rpi.robuboard import set_status_led; set_status_led({r}, {g}, {b})'"
    subprocess.run(command, shell=True)


# run this script with sudo!
def set_status_led(r: int = 50, g: int = 10, b: int = 0, w: int = 0):
    if board_utils.IS_ROBUBOARD_V0:
        from rpi_ws281x import Color, ws, PixelStrip

        status_led = PixelStrip(1, GPIO_STATUS_LED, strip_type=ws.SK6812_STRIP_RGBW)
        status_led.begin()
        status_led.setPixelColor(0, Color(r, g, b, w))
        status_led.show()

    elif board_utils.IS_ROBUBOARD_V1 or board_utils.IS_ROBUBOARD_V3:
        # spi = spidev.SpiDev()
        # spi.open(0, 0)
        spi, path = _open_spi_for_led()

        neopixel_spi_write(spi, [[g, r, b]])
        # spi.close()


def set_i2c_power(enabled: bool = True):
    port = 0
    MCP23017_ADDR = 0x21

    IODIRA = 0x00
    GPIOA = 0x12

    with smbus.SMBus(port) as bus:
        iodira = bus.read_byte_data(MCP23017_ADDR, IODIRA)
        iodira &= ~(1 << 7)
        bus.write_byte_data(MCP23017_ADDR, IODIRA, iodira)

        gpioa = bus.read_byte_data(MCP23017_ADDR, GPIOA)

        if not enabled:
            gpioa |= (1 << 7)
        else:
            gpioa &= ~(1 << 7)

        bus.write_byte_data(MCP23017_ADDR, GPIOA, gpioa)


if __name__ == '__main__':
    print(sys.argv)
    if is_robuboard():
        power_on_teensy()
