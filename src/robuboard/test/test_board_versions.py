"""Verify board detection and Teensy control without Raspberry Pi hardware."""

import importlib
import importlib.util
import subprocess
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

import pytest


class FakeBus:
    """Model the expander registers and record all writes."""

    def __init__(self, hardware, port):
        """Attach this connection to the simulated hardware."""
        self.hardware = hardware
        hardware.ports.append(port)

    def __enter__(self):
        """Return the connection."""
        return self

    def __exit__(self, *args):
        """Record connection cleanup."""
        self.hardware.closed += 1

    def read_byte_data(self, address, register):
        """Read a register, using the power-on default when unset."""
        return self.hardware.registers.get((address, register), 0xFF)

    def write_byte_data(self, address, register, value):
        """Record and apply a register write unless failure is requested."""
        if self.hardware.fail_write:
            raise OSError('I2C write failed')
        self.hardware.writes.append((address, register, value))
        self.hardware.registers[address, register] = value


@pytest.fixture
def board(monkeypatch):
    """Load the actual modules with mocked GPIO, SPI and I2C devices."""
    package = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(package))
    hardware = SimpleNamespace(
        registers={}, writes=[], ports=[], closed=0, fail_write=False,
    )
    smbus = ModuleType('smbus2')
    smbus.SMBus = Mock(side_effect=lambda port: FakeBus(hardware, port))
    smbus.i2c_msg = Mock()
    smbus.smbus2 = smbus
    monkeypatch.setitem(sys.modules, 'smbus2', smbus)
    monkeypatch.setitem(sys.modules, 'lgpio', Mock())
    monkeypatch.setitem(sys.modules, 'spidev', Mock())
    led_module = ModuleType('neopixel.common.neopixel_spi_write')
    led_module.neopixel_spi_write = Mock()
    monkeypatch.setitem(sys.modules, led_module.__name__, led_module)

    def load_module(name, filename):
        spec = importlib.util.spec_from_file_location(
            name, package / 'robuboard' / 'rpi' / filename,
        )
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, name, module)
        spec.loader.exec_module(module)
        return module

    gpio_info = SimpleNamespace(
        stdout='gpiochip0 "GPIO2" "GPIO3" "GPIO17" "GPIO27"',
    )
    with patch.object(Path, 'exists', return_value=False), \
            patch.object(subprocess, 'run', return_value=gpio_info):
        utils = load_module('robuboard.rpi.utils', 'utils.py')
    rpi_package = importlib.import_module('robuboard.rpi')
    monkeypatch.setattr(rpi_package, 'utils', utils, raising=False)
    control = load_module('robuboard.rpi.robuboard', 'robuboard.py')
    monkeypatch.setattr(rpi_package, 'robuboard', control, raising=False)
    teensy_detection = utils.is_mmteensy
    monkeypatch.setattr(utils, 'is_raspberry_pi', lambda: True)
    monkeypatch.setattr(utils, 'i2c_ping', lambda port, address: False)
    monkeypatch.setattr(utils, 'is_mmteensy', lambda: False)
    monkeypatch.setattr(control, '_gpio_write_once', Mock())
    monkeypatch.setattr(control.time, 'sleep', Mock())
    monkeypatch.setattr(
        control, 'is_bootloader_teensy', Mock(return_value=False),
    )
    return SimpleNamespace(
        utils=utils, control=control, hardware=hardware, led=led_module,
        teensy_detection=teensy_detection,
    )


@pytest.mark.parametrize('pi,addresses,usb,version', [
    (False, {0x20}, True, None),
    (True, {0x20}, False, 3),
    (True, {0x20, 0x41}, True, 3),
    (True, {0x41}, False, 1),
    (True, set(), True, 0),
    (True, set(), False, None),
])
def test_detection(board, monkeypatch, pi, addresses, usb, version):
    """Recognize only the matching version and allow an offline Teensy."""
    utils = board.utils
    ping = Mock(side_effect=lambda port, address: address in addresses)
    teensy = Mock(return_value=usb)
    monkeypatch.setattr(utils, 'is_raspberry_pi', lambda: pi)
    monkeypatch.setattr(utils, 'i2c_ping', ping)
    monkeypatch.setattr(utils, 'is_mmteensy', teensy)

    assert utils.is_robuboard() is (version is not None)
    assert utils.IS_ROBUBOARD_V0 is (version == 0)
    assert utils.IS_ROBUBOARD_V1 is (version == 1)
    assert utils.IS_ROBUBOARD_V3 is (version == 3)
    assert all(call.args[0] == 1 for call in ping.call_args_list)
    if not pi or version in (1, 3):
        teensy.assert_not_called()


def test_detection_clears_previous_version(board, monkeypatch):
    """Discard flags from the previous detection result."""
    addresses = {0x20}
    monkeypatch.setattr(
        board.utils, 'i2c_ping', lambda port, address: address in addresses,
    )
    assert board.utils.is_robuboard_v3()
    addresses.clear()
    addresses.add(0x41)
    assert board.utils.is_robuboard_v1()
    assert not board.utils.IS_ROBUBOARD_V3
    addresses.clear()
    assert not board.utils.is_robuboard()
    assert not board.utils.IS_ROBUBOARD_V1


@pytest.mark.parametrize('usb_output,expected', [
    ('Bus 001 Device 001: Linux Foundation root hub', False),
    ('Teensyduino Serial', True),
    ('Teensy Halfkay', True),
    ('NXP Semiconductors SE Blank RT Family', True),
])
def test_teensy_usb_detection(board, usb_output, expected):
    """Require a matching USB identity instead of a nonempty string."""
    with patch.object(
        subprocess, 'run', return_value=SimpleNamespace(stdout=usb_output),
    ):
        assert board.teensy_detection() is expected


def select_version(board, monkeypatch, version):
    """Select the devices that identify one board version."""
    address = {0: None, 1: 0x41, 3: 0x20}[version]
    monkeypatch.setattr(
        board.utils, 'i2c_ping', lambda port, candidate: candidate == address,
    )
    monkeypatch.setattr(board.utils, 'is_mmteensy', lambda: version == 0)


@pytest.mark.parametrize('config', [0xFF, 0x00])
def test_v3_initialization_preserves_usb_and_switch_inputs(
        board, monkeypatch, config):
    """Preload LOW, protect switch inputs, and retain USB pin settings."""
    select_version(board, monkeypatch, 3)
    board.hardware.registers[0x20, 0x03] = config
    board.control.init_gpios()

    assert board.hardware.writes == [
        (0x20, 0x01, 0xD7),
        (0x20, 0x03, (config | 0x14) & ~0x28),
    ]
    assert board.hardware.closed == 1
    assert board.hardware.ports == [1]
    board.control._gpio_write_once.assert_called_once_with(26, 1)
    board.control.init_gpios()
    assert len(board.hardware.writes) == 2


def test_initialization_failure_can_be_retried(board, monkeypatch):
    """Leave initialization pending and close the bus after an I2C error."""
    select_version(board, monkeypatch, 3)
    board.hardware.fail_write = True
    with pytest.raises(OSError, match='I2C write failed'):
        board.control.init_gpios()
    assert not board.control.robuboard_init_gpios
    assert board.hardware.closed == 1
    board.control._gpio_write_once.assert_not_called()
    board.hardware.fail_write = False
    board.control.init_gpios()
    assert board.control.robuboard_init_gpios


@pytest.mark.parametrize(
    'version,address,onoff_bit', [(1, 0x41, 0), (3, 0x20, 3)],
)
@pytest.mark.parametrize(
    'power_on,durations', [(False, [5]), (True, [5, 1.0])],
)
def test_expander_power_pulses(
        board, monkeypatch, version, address, onoff_bit, power_on, durations):
    """Pulse the correct ON/OFF pin and keep unrelated outputs intact."""
    select_version(board, monkeypatch, version)
    board.control.init_gpios()
    # USB-C enable outputs remain high throughout Teensy operations.
    output = 0x82 if version == 3 else 0x0C
    board.hardware.registers[address, 0x01] = output
    board.hardware.writes.clear()
    operation = (
        board.control.power_on_teensy if power_on
        else board.control.power_off_teensy
    )
    operation()

    assert board.hardware.writes == [
        write for _ in durations for write in (
            (address, 0x01, output | (1 << onoff_bit)),
            (address, 0x01, output),
        )
    ]
    delays = [call.args[0] for call in board.control.time.sleep.call_args_list]
    assert delays == durations
    assert all(
        call.args[0] != 23
        for call in board.control._gpio_write_once.call_args_list
    )


@pytest.mark.parametrize(
    'version,address,boot_bit', [(1, 0x41, 1), (3, 0x20, 5)],
)
@pytest.mark.parametrize('already_booting,force,pulse', [
    (False, False, True), (True, False, False), (True, True, True),
])
def test_expander_bootloader_without_usb(
        board, monkeypatch, version, address, boot_bit,
        already_booting, force, pulse):
    """Boot through the expander with no USB device and honor force."""
    select_version(board, monkeypatch, version)
    board.control.init_gpios()
    board.hardware.registers[address, 0x01] = 0x82 if version == 3 else 0x0C
    output = board.hardware.registers[address, 0x01]
    board.hardware.writes.clear()
    monkeypatch.setattr(
        board.control, 'is_mmteensy', Mock(return_value=False),
    )
    board.control.is_bootloader_teensy.return_value = already_booting
    board.control.start_bootloader_teensy(force)

    expected = [
        (address, 0x01, output | (1 << boot_bit)), (address, 0x01, output),
    ]
    assert board.hardware.writes == (expected if pulse else [])
    board.control.is_mmteensy.assert_not_called()


def test_v0_retains_gpio_control(board, monkeypatch):
    """Keep the original GPIO23 power sequence for V0."""
    select_version(board, monkeypatch, 0)
    board.control.power_on_teensy()
    reset_values = [
        call.args[1] for call in board.control._gpio_write_once.call_args_list
        if call.args[0] == 23
    ]
    assert reset_values == [0, 1, 0, 1, 0]
    assert board.hardware.writes == []


def test_interrupted_pulse_releases_only_control_pin(board, monkeypatch):
    """Release ON/OFF and retain newer USB settings after interruption."""
    select_version(board, monkeypatch, 3)
    board.control.init_gpios()
    board.hardware.registers[0x20, 0x01] = 0x00
    board.hardware.writes.clear()

    def interrupt(duration):
        # Another operation enables USB-C while the Teensy pulse is active.
        board.hardware.registers[0x20, 0x01] |= 0x82
        raise KeyboardInterrupt

    monkeypatch.setattr(board.control.time, 'sleep', interrupt)
    with pytest.raises(KeyboardInterrupt):
        board.control.power_off_teensy()
    assert board.hardware.writes == [(0x20, 0x01, 0x08), (0x20, 0x01, 0x82)]
    assert board.hardware.closed == 2


def test_v3_status_led_uses_spi(board, monkeypatch):
    """Use the existing SPI LED driver for the replacement expander board."""
    select_version(board, monkeypatch, 3)
    board.control.init_gpios()
    spi = Mock()
    monkeypatch.setattr(
        board.control, '_open_spi_for_led', lambda: (spi, '/dev/spidev1.0'),
    )
    board.control.set_status_led(10, 20, 30)
    board.led.neopixel_spi_write.assert_called_once_with(spi, [[20, 10, 30]])


@pytest.fixture
def commands(board, monkeypatch):
    """Load ROS entry points with a simulated ROS node and messages."""
    rclpy = ModuleType('rclpy')
    rclpy.init = Mock()
    rclpy.shutdown = Mock()
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    for name in ('node', 'publisher', 'qos', 'timer'):
        module = ModuleType('rclpy.' + name)
        setattr(rclpy, name, module)
        monkeypatch.setitem(sys.modules, module.__name__, module)
    rclpy.node.Node = object
    messages = ModuleType('std_msgs.msg')
    messages.ByteMultiArray = Mock()
    messages.MultiArrayDimension = Mock()
    monkeypatch.setitem(sys.modules, messages.__name__, messages)
    spec = importlib.util.spec_from_file_location(
        'robuboard.gpioctrl',
        Path(board.control.__file__).parents[1] / 'gpioctrl.py',
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = Mock()
    monkeypatch.setattr(rclpy.node, 'Node', Mock(return_value=node))
    return SimpleNamespace(module=module, node=node, rclpy=rclpy)


@pytest.mark.parametrize('version', [0, 1, 3])
def test_ros_version_command(board, commands, monkeypatch, version):
    """Report the detected hardware version through the ROS command."""
    select_version(board, monkeypatch, version)
    commands.module.main_is_robuboard()
    commands.node.get_logger().info.assert_called_once_with(
        f'RobuBoard Yes/V{version}',
    )
    commands.node.destroy_node.assert_called_once()
    commands.rclpy.shutdown.assert_called_once()


def test_ros_bootloader_command_without_usb(board, commands, monkeypatch):
    """Allow the ROS command to reach P5 when the Teensy is absent from USB."""
    select_version(board, monkeypatch, 3)
    commands.node.get_parameter().get_parameter_value().bool_value = False
    commands.module.main_start_bootloader_teensy()
    assert board.hardware.writes[-2:] == [
        (0x20, 0x01, 0xF7), (0x20, 0x01, 0xD7),
    ]
    commands.node.get_logger().error.assert_not_called()
