"""Keep board power on during reboot; exercise shutdown with simulated GPIO."""

import importlib.util
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


@pytest.fixture
def poweroff(monkeypatch):
    """Load the actual shutdown entry point without Raspberry Pi hardware."""
    path = (Path(__file__).resolve().parents[1] / 'robuboard' / 'rpi'
            / 'robuboard_power_off_5v.py')
    spec = importlib.util.spec_from_file_location('robuboard_poweroff_test', path)
    module = importlib.util.module_from_spec(spec)
    # Importing the module itself must not import or initialize GPIO.
    monkeypatch.setitem(sys.modules, 'RPi', None)
    monkeypatch.setitem(sys.modules, 'RPi.GPIO', None)
    monkeypatch.setitem(sys.modules, 'lgpio', None)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.os, 'sync', Mock())
    monkeypatch.setattr(module.time, 'sleep', Mock())
    return module


@pytest.mark.parametrize('jobs', [
    '11 reboot.target start waiting\n12 shutdown.target start running\n',
    '11 kexec.target start waiting\n',
    '11 soft-reboot.target start waiting\n',
    '11 systemd-reboot.service start waiting\n',
    '11 systemd-kexec.service start waiting\n',
    '11 systemd-soft-reboot.service start waiting\n',
    '11 shutdown.target start waiting\n',
    '11 poweroff.target stop waiting\n',
    '11 robuboard_power_off_5v.service start running\n',
    '',
    'not a job\n',
    # A reboot job must prevent power-cut even with conflicting stale jobs.
    '11 poweroff.target start waiting\n12 reboot.target start waiting\n',
    '11 halt.target start waiting\n12 systemd-reboot.service start waiting\n',
])
def test_other_actions_never_touch_gpio(poweroff, monkeypatch, jobs):
    """No shutdown confirmation means no GPIO import, sync or power-cut."""
    monkeypatch.setattr(
        poweroff.subprocess, 'run', Mock(return_value=SimpleNamespace(stdout=jobs)),
    )
    poweroff.main()
    poweroff.os.sync.assert_not_called()
    poweroff.time.sleep.assert_not_called()


@pytest.mark.parametrize('error', [
    OSError('systemctl unavailable'),
    subprocess.CalledProcessError(1, ['systemctl']),
    subprocess.TimeoutExpired(['systemctl'], 2),
])
def test_unknown_action_keeps_power_on(poweroff, monkeypatch, error):
    """Missing systemd access must not accidentally power down the board."""
    monkeypatch.setattr(poweroff.subprocess, 'run', Mock(side_effect=error))
    poweroff.main()
    poweroff.os.sync.assert_not_called()
    poweroff.time.sleep.assert_not_called()


@pytest.mark.parametrize('target', ['poweroff.target', 'halt.target'])
def test_confirmed_shutdown_cuts_power(poweroff, monkeypatch, target):
    """Keep the actual HIGH-to-LOW sequence for poweroff and halt."""
    query = Mock(return_value=SimpleNamespace(
        stdout=f'11 {target} start waiting\n12 shutdown.target start running\n',
    ))
    monkeypatch.setattr(poweroff.subprocess, 'run', query)
    gpio = Mock()
    gpio.gpiochip_open.return_value = 77
    monkeypatch.setitem(sys.modules, 'lgpio', gpio)
    monkeypatch.setattr(poweroff, 'find_header_gpiochip', Mock(return_value=4))
    poweroff.main()
    query.assert_called_once_with(
        ['/usr/bin/systemctl', '--system', 'list-jobs',
         '--no-legend', '--no-pager', '--plain'],
        capture_output=True, text=True, check=True, timeout=2,
    )
    poweroff.os.sync.assert_called_once_with()
    poweroff.find_header_gpiochip.assert_called_once_with(gpio)
    gpio.gpiochip_open.assert_called_once_with(4)
    gpio.gpio_claim_output.assert_called_once_with(77, 26, 1)
    gpio.gpio_write.assert_called_once_with(77, 26, 0)
    gpio.gpiochip_close.assert_not_called()


@pytest.mark.parametrize('chip', [0, 4, 10])
def test_header_discovery_does_not_claim_gpio(poweroff, monkeypatch, chip):
    """Find CM4/CM5 GPIO by metadata, independent of chip numbering."""
    paths = [Path('/dev/gpiochip1'), Path(f'/dev/gpiochip{chip}')]
    monkeypatch.setattr(Path, 'glob', lambda *args: paths)
    gpio = Mock()
    gpio.gpiochip_open.side_effect = lambda index: index + 100
    gpio.gpio_get_chip_info.side_effect = lambda handle: [
        0, 54 if handle == chip + 100 else 4, f'gpiochip{handle - 100}', 'label',
    ]
    gpio.gpio_get_line_info.side_effect = lambda handle, pin: [
        0, pin, 0, f'GPIO{pin}', '',
    ]
    assert poweroff.find_header_gpiochip(gpio) == chip
    assert gpio.gpiochip_close.call_count == gpio.gpiochip_open.call_count
    gpio.gpio_claim_output.assert_not_called()
    gpio.gpio_write.assert_not_called()


def test_missing_header_does_not_change_gpio(poweroff, monkeypatch):
    """Refuse to use another controller with a GPIO at offset 26."""
    monkeypatch.setattr(Path, 'glob', lambda *args: [Path('/dev/gpiochip0')])
    gpio = Mock()
    gpio.gpiochip_open.return_value = 77
    gpio.gpio_get_chip_info.return_value = [0, 32, 'gpiochip0', 'wrong controller']
    gpio.gpio_get_line_info.return_value = [0, 25, 0, 'different pin', '']
    with pytest.raises(RuntimeError, match='GPIO-Header'):
        poweroff.find_header_gpiochip(gpio)
    gpio.gpiochip_close.assert_called_once_with(77)
    gpio.gpio_claim_output.assert_not_called()
    gpio.gpio_write.assert_not_called()
