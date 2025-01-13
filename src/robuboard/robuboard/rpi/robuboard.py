from robuboard.rpi.utils import is_raspberry_pi

import time
import sys

try:
    import RPi.GPIO as GPIO
except:
    print("The module RPi.GPIO can only run on the raspberry pi!")

GPIO_TEENSY_RESET = 23
GPIO_POWER_SWITCH = 20

global robuboard_init_gpios
global robuboard_force_5v_supply_on

robuboard_init_gpios:bool = False
robuboard_force_5v_supply_on:bool = False

def init_gpios():
    global robuboard_init_gpios
    if not robuboard_init_gpios:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_TEENSY_RESET, GPIO.OUT)
        force_on_5v_supply()
        #GPIO.setup(GPIO_POWER_SWITCH, GPIO.IN)
        robuboard_init_gpios = True

def is_on_5v_supply() -> bool:
    global robuboard_force_5v_supply_on
    return robuboard_force_5v_supply_on


def force_on_5v_supply():
    global robuboard_force_5v_supply_on
    GPIO.setup(GPIO_POWER_SWITCH, GPIO.OUT)
    GPIO.output(GPIO_POWER_SWITCH, GPIO.HIGH)
    robuboard_force_5v_supply_on = True

def config_power_switch_to_input():
    global robuboard_force_5v_supply_on
    GPIO.setup(GPIO_POWER_SWITCH, GPIO.IN)
    robuboard_force_5v_supply_on = False

def get_power_switch() -> bool:
    global robuboard_force_5v_supply_on
    if not robuboard_force_5v_supply_on:
        return GPIO.input(GPIO_POWER_SWITCH)
    return True

def power_off_teensy():
    init_gpios()
    force_on_5v_supply()
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


if __name__ == '__main__':
    print(sys.argv)
    if is_raspberry_pi():
        power_on_teensy()

