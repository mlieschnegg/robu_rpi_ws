from robuboard.rpi.utils import is_raspberry_pi, is_mmteensy, is_robuboard

import time
import sys

try:
    import RPi.GPIO as GPIO
except:
    print("The module RPi.GPIO can only run on the raspberry pi!")

GPIO_TEENSY_RESET = 23
GPIO_POWER_SWITCH = 20
GPIO_POWER_REGULATOR_EN = 26
GPIO_STATUS_LED = 21

global robuboard_init_gpios
global robuboard_enable_5v_supply_on

robuboard_init_gpios:bool = False
robuboard_enable_5v_supply_on:bool = False

def init_gpios():
    global robuboard_init_gpios
    if not robuboard_init_gpios:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_TEENSY_RESET, GPIO.OUT)
        GPIO.output(GPIO_TEENSY_RESET, GPIO.LOW)

        GPIO.setup(GPIO_POWER_REGULATOR_EN, GPIO.OUT)
        enable_5v_supply()
        
        GPIO.setup(GPIO_POWER_SWITCH, GPIO.IN)
        
        robuboard_init_gpios = True

def is_on_5v_supply() -> bool:
    return robuboard_enable_5v_supply_on
    # return GPIO.input(GPIO_POWER_REGULATOR_EN)

def enable_5v_supply():
    global robuboard_enable_5v_supply_on
    GPIO.output(GPIO_POWER_SWITCH, GPIO.HIGH)
    robuboard_enable_5v_supply_on = True

def disable_5v_supply():
    global robuboard_enable_5v_supply_on
    GPIO.output(GPIO_POWER_SWITCH, GPIO.LOW)
    robuboard_enable_5v_supply_on = False

def get_power_switch() -> bool:
    return GPIO.input(GPIO_POWER_SWITCH)

def power_off_teensy():
    init_gpios()
    enable_5v_supply()
    print("powering off teensy...")
    GPIO.output(GPIO_TEENSY_RESET, GPIO.HIGH)
    time.sleep(5)
    GPIO.output(GPIO_TEENSY_RESET, GPIO.LOW)

def power_on_teensy():
    init_gpios()

    power_off_teensy()
    print("powering on teensy...")
    GPIO.output(GPIO_TEENSY_RESET, GPIO.HIGH)
    time.sleep(1.0)
    GPIO.output(GPIO_TEENSY_RESET, GPIO.LOW)
    #GPIO.cleanup()

def start_bootloader_teensy():
    import subprocess
    init_gpios()
    enable_5v_supply()
    print("starting bootloader on teensy...")
    subprocess.run(["teensy_loader_cli", "--mcu=TEENSY_MICROMOD", "-s", "-b"])

def upload_firmware_teensy(firmware_path:str="~/work/robocup24-teensy/.pio/build/teensymm/firmware.hex"):
    import subprocess
    init_gpios()
    enable_5v_supply()
    print("uploading firmware to teensy...")
    subprocess.run(["teensy_loader_cli", "--mcu=TEENSY_MICROMOD", "-s", "-w", firmware_path])

def start_status_led_with_sudo(r:int=255, g:int=255, b:int=51):
    import subprocess
    command = f"sudo python3 -c 'from robuboard.rpi.robuboard import set_status_led; set_status_led({r}, {g}, {b})'"
    subprocess.run(command, shell=True)

#run this script with sudo!
def set_status_led(r:int=255, g:int=255, b:int=51):
    print("Setting status LED!")
    from rpi_ws281x import Adafruit_NeoPixel, Color, ws
    status_led = Adafruit_NeoPixel(1, GPIO_STATUS_LED, strip_type=ws.WS2811_STRIP_GRB)  
    status_led.begin()
    status_led.setPixelColor(0, Color(r, g, b))
    status_led.show()
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_STATUS_LED, GPIO.IN)

if __name__ == '__main__':
    # start_status_led_with_sudo()
    print(sys.argv)
    if is_robuboard():
        power_on_teensy()

