#!/usr/bin/env python3
import os
import time
import RPi.GPIO as GPIO

GPIO_POWER_REGULATOR_EN = 26  # BCM GPIO26
#TODO: blink led-light
def main():
    # Datei-Systeme flushen (extra Sicherheit)
    os.sync()
    time.sleep(0.2)

    # BCM-Nummerierung verwenden
    GPIO.setmode(GPIO.BCM)

    # Pin als Output setzen. Wir setzen initial HIGH (falls Pin bisher float war),
    # und schalten dann sicher auf LOW.
    GPIO.setup(GPIO_POWER_REGULATOR_EN, GPIO.OUT, initial=GPIO.HIGH)

    time.sleep(0.05)

    # Regler abschalten
    GPIO.output(GPIO_POWER_REGULATOR_EN, GPIO.LOW)
    time.sleep(1.0)

    GPIO.cleanup()

if __name__ == "__main__":
    main()
