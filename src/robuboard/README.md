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

Run the hardware-independent regression tests with:

```sh
python3 -m pytest src/robuboard/test/test_board_versions.py
```
