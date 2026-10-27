#!/usr/bin/env python3
"""Cut the board supply on a confirmed systemd poweroff/halt, never on reboot."""

import os
from pathlib import Path
import subprocess
import time

GPIO_POWER_REGULATOR_EN = 26  # BCM GPIO26, not a gpiochip number


def is_poweroff_requested():
    """Nur einen expliziten Poweroff/Halt-Auftrag zum Abschalten akzeptieren."""
    try:
        result = subprocess.run(
            ['/usr/bin/systemctl', '--system', 'list-jobs',
             '--no-legend', '--no-pager', '--plain'],
            capture_output=True, text=True, check=True, timeout=2,
        )
    except (OSError, subprocess.SubprocessError) as error:
        # Wenn systemd nicht mehr erreichbar ist, die Versorgung erhalten.
        print(f'Shutdown-Art nicht feststellbar: {error}', flush=True)
        return False

    # poweroff.target ist beim Start dieses Dienstes noch nicht aktiv, sondern
    # wartet als Start-Job. Deshalb genügt "systemctl is-active" hier nicht.
    start_jobs = set()
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) >= 3 and fields[2] == 'start':
            start_jobs.add(fields[1])

    # Reboot hat im Zweifelsfall Vorrang, auch bei veralteten .wants-Symlinks.
    if start_jobs & {'reboot.target', 'kexec.target', 'soft-reboot.target',
                     'systemd-reboot.service', 'systemd-kexec.service',
                     'systemd-soft-reboot.service'}:
        return False
    return bool(start_jobs & {'poweroff.target', 'halt.target'})


def find_header_gpiochip(gpio):
    """Den GPIO-Header über Line-Namen finden, ohne Pins anzufordern."""
    # CM4 und CM5 verwenden unterschiedliche Controller; beim CM5 ist GPIO26
    # hier auf gpiochip4/RP1. Die Nummer kann sich nach Kernel-Updates ändern.
    for path in sorted(Path('/dev').glob('gpiochip[0-9]*')):
        chip = int(path.name.removeprefix('gpiochip'))
        handle = gpio.gpiochip_open(chip)
        try:
            # lgpio liefert [Status, Anzahl Lines, Name, Label] bzw.
            # [Status, Offset, Flags, Line-Name, Consumer]. Nur Metadaten lesen.
            if (gpio.gpio_get_chip_info(handle)[1] > GPIO_POWER_REGULATOR_EN
                    and gpio.gpio_get_line_info(handle, 25)[3] == 'GPIO25'
                    and gpio.gpio_get_line_info(handle, 26)[3] == 'GPIO26'):
                return chip
        finally:
            gpio.gpiochip_close(handle)
    raise RuntimeError('GPIO-Header mit GPIO25/GPIO26 nicht gefunden; keine Pins verändert.')


def main():
    if not is_poweroff_requested():
        print('Kein Poweroff/Halt-Auftrag: GPIO26 und 5V-Versorgung bleiben unverändert.',
              flush=True)
        return

    # RPi.GPIO unterstützt den CM5/RP1 nicht. lgpio ist im Robuboard-Setup
    # bereits installiert und benötigt weder ROS noch eine aktivierte Venv.
    import lgpio

    chip = find_header_gpiochip(lgpio)
    os.sync()
    time.sleep(0.2)

    handle = lgpio.gpiochip_open(chip)
    # HIGH bereits beim Anfordern setzen, damit kein kurzer LOW-Impuls entsteht.
    lgpio.gpio_claim_output(handle, GPIO_POWER_REGULATOR_EN, 1)
    time.sleep(0.05)

    print(f'Poweroff/Halt bestätigt: GPIO26 auf gpiochip{chip} schaltet 5V ab.', flush=True)
    lgpio.gpio_write(handle, GPIO_POWER_REGULATOR_EN, 0)
    time.sleep(1.0)
    # Bis zum Ende des Prozesses angefordert lassen. Kein Freigeben/Umstellen
    # auf Input, solange die Versorgung noch nicht abgefallen ist.


if __name__ == '__main__':
    main()
