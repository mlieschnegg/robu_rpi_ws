#!/usr/bin/env python3
import os
import time
import RPi.GPIO as GPIO
import sys

GPIO_POWER_REGULATOR_EN = 26  # BCM GPIO26
GPIO_POWER_SWITCH = 25

TIMEOUT_POWEROFF_OFF = 20.0

def main():
    # Datei-Systeme flushen (extra Sicherheit)
    os.sync()
    time.sleep(0.2)

    # BCM-Nummerierung verwenden
    GPIO.setmode(GPIO.BCM)

    # Pin als Output setzen. Wir setzen initial HIGH (falls Pin bisher float war),
    # und schalten dann sicher auf LOW.
    GPIO.setup(GPIO_POWER_REGULATOR_EN, GPIO.OUT, initial=GPIO.HIGH)

    GPIO.setup(GPIO_POWER_SWITCH, GPIO.IN)
        
    time.sleep(0.05)

    # power off device
    if GPIO.input(GPIO_POWER_SWITCH):
        shutdown = False
        while not shutdown:
            print("Release the power button to shut down the RobuBoard.")
            while GPIO.input(GPIO_POWER_SWITCH):
                time.sleep(0.1)

            print(f"Shutdown initiated ({int(TIMEOUT_POWEROFF_OFF):02d} s)")
            shutdown = True
            for i in range(int(TIMEOUT_POWEROFF_OFF), 0, -1):
                sys.stdout.write(f"\rShutdown in {i:02d} s")
                sys.stdout.flush()
                if GPIO.input(GPIO_POWER_SWITCH):
                    print(f"\nShutdown has been retriggered.")
                    shutdown = False
                    break
                time.sleep(1.0)

        print() #\n
        time.sleep(TIMEOUT_POWEROFF_OFF - int(TIMEOUT_POWEROFF_OFF))

    # Regler abschalten
    print("POWROFF!")
    GPIO.output(GPIO_POWER_REGULATOR_EN, GPIO.LOW)
    time.sleep(1.0)
    # Kein cleanup() machen, das könnte den Regler wieder einschalten, sobald die Pins wieder
    # auf Standardeinstellung sind
    # GPIO.cleanup()

if __name__ == "__main__":
    main()
