from robuboard.rpi.utils import (
    is_raspberry_pi,
    is_mmteensy,
    is_robuboard,
    is_robuboard_v1,
    is_bootloader_teensy,
    is_serial_teensy,
)
from robuboard.rpi.utils import IS_ROBUBOARD_V1, IS_ROBUBOARD
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

robuboard_init_gpios: bool = False
robuboard_enable_5v_supply_on: bool = False


def _gpio_write_once(pin: int, value: int, retries: int = 3, delay: float = 0.01):
    """
    Claim output temporarily, write value, then release.
    Retries if GPIO is busy.
    """
    last_error = None

    for attempt in range(retries):
        handle = lgpio.gpiochip_open(0)
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
        handle = lgpio.gpiochip_open(0)
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
        is_robuboard_v1()

        if not IS_ROBUBOARD_V1:
            # Teensy reset inactive default
            _gpio_write_once(GPIO_TEENSY_RESET, 0)
        else:
            try:
                bus = smbus.SMBus(1)
                # Configure pins 0,1 as outputs; 2,3 as inputs (0x0C)
                bus.write_byte_data(PCA9536_ADDR, PCA9536_REG_CONFIG, 0x0C)

                # Initialize outputs (pins 0 and 1) to LOW
                bus.write_byte_data(PCA9536_ADDR, PCA9536_REG_OUTPUT, 0x00)
                bus.close()

            except Exception as e:
                print(f"Failed to configure PCA9536: {e}")

        # 5V default enable
        enable_5v_supply()

        robuboard_init_gpios = True


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

    if not IS_ROBUBOARD_V1:
        _gpio_write_once(GPIO_TEENSY_RESET, 1)
        time.sleep(5)
        _gpio_write_once(GPIO_TEENSY_RESET, 0)
    else:
        bus = smbus.SMBus(1)
        val = bus.read_byte_data(PCA9536_ADDR, PCA9536_REG_OUTPUT)
        print(f"val (on): {val | (1 << PCA9536_BIT_TEENSY_RESET):b}")
        bus.write_byte_data(
            PCA9536_ADDR,
            PCA9536_REG_OUTPUT,
            val | (1 << PCA9536_BIT_TEENSY_RESET),
        )
        time.sleep(5)
        print(f"val (off) {val & ~(1 << PCA9536_BIT_TEENSY_RESET):b}")
        bus.write_byte_data(
            PCA9536_ADDR,
            PCA9536_REG_OUTPUT,
            val & ~(1 << PCA9536_BIT_TEENSY_RESET),
        )
        bus.close()


def power_on_teensy():
    init_gpios()

    power_off_teensy()
    print("powering on teensy...")

    if not IS_ROBUBOARD_V1:
        _gpio_write_once(GPIO_TEENSY_RESET, 1)
        time.sleep(1.0)
        _gpio_write_once(GPIO_TEENSY_RESET, 0)
    else:
        bus = smbus.SMBus(1)
        val = bus.read_byte_data(PCA9536_ADDR, PCA9536_REG_OUTPUT)
        print(f"val (on): {val | (1 << PCA9536_BIT_TEENSY_RESET):b}")
        bus.write_byte_data(
            PCA9536_ADDR,
            PCA9536_REG_OUTPUT,
            val | (1 << PCA9536_BIT_TEENSY_RESET),
        )
        time.sleep(1)
        print(f"val (off): {val & ~(1 << PCA9536_BIT_TEENSY_RESET):b}")
        bus.write_byte_data(
            PCA9536_ADDR,
            PCA9536_REG_OUTPUT,
            val & ~(1 << PCA9536_BIT_TEENSY_RESET),
        )
        bus.close()


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

    if is_mmteensy() and (not is_bootloader_teensy() or force):
        print("starting bootloader on teensy...")

        if IS_ROBUBOARD_V1 and not is_bootloader_teensy():
            bus = smbus.SMBus(1)
            val = bus.read_byte_data(PCA9536_ADDR, PCA9536_REG_OUTPUT)
            print(f"val (on): {val | (1 << PCA9536_BIT_TEENSY_BOOT):b}")
            bus.write_byte_data(
                PCA9536_ADDR,
                PCA9536_REG_OUTPUT,
                val | (1 << PCA9536_BIT_TEENSY_BOOT),
            )
            time.sleep(0.1)
            print(f"val (off): {val & ~(1 << PCA9536_BIT_TEENSY_BOOT):b}")
            bus.write_byte_data(
                PCA9536_ADDR,
                PCA9536_REG_OUTPUT,
                val & ~(1 << PCA9536_BIT_TEENSY_BOOT),
            )
            bus.close()
        elif not is_bootloader_teensy():
            firmware_path: str = "/home/robu/work/robocup/robocup-teensy/.pio/build/teensymm/firmware.hex"
            subprocess.run(
                ["teensy_loader_cli", "--mcu=TEENSY_MICROMOD", "-s", firmware_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    elif is_bootloader_teensy():
        print("bootloader allready activated!")
    else:
        print("invalid state of teensy! Press boot switch!")


def upload_firmware_teensy(
    firmware_path: str = "/home/robu/work/robocup/robocup-teensy/.pio/build/teensymm/firmware.hex"
):
    init_gpios()
    enable_5v_supply()
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
    if IS_ROBUBOARD and not IS_ROBUBOARD_V1:
        from rpi_ws281x import Color, ws, PixelStrip

        status_led = PixelStrip(1, GPIO_STATUS_LED, strip_type=ws.SK6812_STRIP_RGBW)
        status_led.begin()
        status_led.setPixelColor(0, Color(r, g, b, w))
        status_led.show()

    elif IS_ROBUBOARD_V1:
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