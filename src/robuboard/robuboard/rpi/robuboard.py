from robuboard.rpi.utils import is_raspberry_pi

import time
import sys

try:
    import RPi.GPIO as GPIO
except:
    print("The module RPi.GPIO can only run on the raspberry pi!")

GPIO_TEENSY_RESET = 23
GPIO_POWER_SWITCH = 20
GPIO_POWER_REGULATOR_EN = 26

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


if __name__ == '__main__':
    print(sys.argv)
    if is_raspberry_pi():
        power_on_teensy()

