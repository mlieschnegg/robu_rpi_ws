# RobuBoard hardware versions

The package detects the board on Raspberry Pi using I2C bus 1:

| Hardware | Detection | Teensy ON/OFF | Teensy BOOT |
| --- | --- | --- | --- |
| V0 | Teensy detected over USB, neither expander present | GPIO23 | `teensy_loader_cli` |
| V1 | PCA9536 at `0x41` | P0 | P1 |
| V3 | PCAL6408A at `0x20` | P3 (`TEENSY_ONOFF`) | P5 (`TEENSY_BOOT`) |

V3 takes precedence if both addresses respond. V1 and V3 can be detected even
when the Teensy is powered off or absent from USB. Use `is_robuboard_v3()` from
`robuboard.rpi.utils`, or `ros2 run robuboard is_robuboard`, to check V3.
The systemd detection script also accepts V1 and V3.

On V3, P3 and P5 drive MOSFET gates: HIGH activates the respective control,
LOW releases it. Initialization preloads LOW before configuring the outputs.
P2 (`SW_TEENSY_ONOFF`) and P4 (`SW_TEENSY_BOOT`) remain inputs. The USB-C pins
P0/P1/P6/P7 retain their existing output values and directions.
The output and direction registers follow the
[PCAL6408A datasheet](https://www.nxp.com/docs/en/data-sheet/PCAL6408A.pdf).

The existing commands use the detected version automatically:

- `ros2 run robuboard poweroff_teensy`: ON/OFF HIGH for 5 seconds, then LOW.
- `ros2 run robuboard poweron_teensy` or `reset_teensy`: power-off sequence,
  then ON/OFF HIGH for 1 second and LOW again.
- `ros2 run robuboard start_bootloader_teensy`: BOOT HIGH for 0.1 seconds,
  then LOW. Use `--ros-args -p force:=true` to pulse an already active bootloader.

V3 uses the SPI status LED path, as V1 does. GPIO25 and GPIO26 retain the
existing board power-switch and 5 V supply functions.

The `robuboard_power_off_5v.service` cuts GPIO26 only when systemd has a pending
start job for `poweroff.target` or `halt.target`. Reboot, kexec, an ordinary
manual service start, and an unreadable shutdown state leave the GPIO untouched.
The shutdown script checks this before importing GPIO. The service runs after
normal services have stopped and before systemd performs poweroff/halt.
It uses the system `lgpio` package and finds the header controller by its
`GPIO25`/`GPIO26` line names, rather than assuming a gpiochip number. The old
`RPi.GPIO` implementation failed on CM5 with "Cannot determine SOC peripheral
base address".

The systemd guard only prevents this script from cutting power during reboot.
On the inspected CM5, GPIO26 belongs to RP1 (`gpiochip4`); a controller or
bootloader reset may still interrupt the hardware power-hold signal. The
`gpio=26=op,dh` boot setting is not a guarantee of a continuous HIGH level
through reset. Diagnose that separately by observing GPIO26 and regulator EN
during a controlled reboot; changing the shutdown unit alone cannot establish
whether this hardware signal stays asserted.

During the controlled `sudo reboot now` test on 2026-10-27 (CM5 Lite,
Ubuntu kernel `6.8.0-1065-raspi`), systemd reached `reboot.target` without
running this service. Both supply LEDs went out and the board required manual
power-on, even with the guarded unit installed. The reboot power loss therefore
remains unresolved outside this service. In the RobuBoard schematic, GPIO26
connects through R44 (1 kOhm) to `POWER_EN`, which feeds the regulator enable
path through R12/D4. Observe GPIO26, `POWER_EN` and regulator EN during reset
to distinguish loss of the hold signal from a regulator/protection fault;
the waveform has not yet been measured. The firmware
[`gpio` boot setting](https://www.raspberrypi.com/documentation/computers/config_txt.html#gpio)
is applied during boot and does not guarantee an uninterrupted signal through
reset.

After updating the repository on the RobuBoard, apply the unit changes and
replace any old enablement links without starting the power-cut service:

```sh
sudo systemctl daemon-reload
sudo systemctl reenable ~/work/.robu/config/service/robuboard_power_off_5v.service
```

`rpi_setup.sh` also recreates these links when run again.

Run the hardware-independent regression tests with:

```sh
python3 -m pytest src/robuboard/test/test_board_versions.py src/robuboard/test/test_power_off_5v.py
```
