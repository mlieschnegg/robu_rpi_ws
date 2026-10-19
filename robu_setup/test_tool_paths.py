"""Test profile migrations and tool links using temporary files only."""

import contextlib
import io
import os
import shlex
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from setup_tool_paths import install_shell_config, link_platformio_commands

REPO_DIR = Path(__file__).resolve().parents[1]
FRAGMENT = REPO_DIR / 'config' / 'shell' / 'tool_paths.sh'


class ToolPathTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.bashrc = self.root / '.bashrc'
        self.core = self.root / 'core' / 'bin'
        self.launchers = self.root / 'local' / 'bin'
        self.output = io.StringIO()
        redirect = contextlib.redirect_stdout(self.output)
        redirect.__enter__()
        self.addCleanup(redirect.__exit__, None, None, None)

    def create_command(self, name: str) -> Path:
        self.core.mkdir(parents=True, exist_ok=True)
        command = self.core / name
        command.write_text('#!/bin/bash\necho "isolated core"\n')
        command.chmod(0o755)
        return command

    def test_legacy_migration_preserves_settings_and_original_backup(self) -> None:
        original = (
            '# my shell settings\nalias robot="ros2 topic list"\n'
            'export PATH="$HOME/.platformio/penv/bin:$PATH"\n'
            'export PATH="$HOME/.local/bin:$PATH"\n'
            'export PATH="$HOME/custom/bin:$PATH"\n'
        )
        self.bashrc.write_text(original)
        self.bashrc.chmod(0o640)
        install_shell_config(self.bashrc, REPO_DIR)
        first = self.bashrc.read_text()
        self.assertIn('alias robot="ros2 topic list"', first)
        self.assertIn('export PATH="$HOME/custom/bin:$PATH"', first)
        self.assertNotIn('.platformio/penv/bin', first)
        self.assertEqual(first.count('# >>> ROBU tool paths >>>'), 1)
        self.assertEqual(self.bashrc.stat().st_mode & 0o777, 0o640)
        backup = self.bashrc.with_name('.bashrc.pre-robu-tool-paths.bak')
        self.assertEqual(backup.read_text(), original)
        install_shell_config(self.bashrc, REPO_DIR)
        self.assertEqual(self.bashrc.read_text(), first)
        self.assertEqual(backup.read_text(), original)

    def test_manual_repair_block_is_migrated_to_shared_config(self) -> None:
        self.bashrc.write_text('export ROS_LOCALHOST_ONLY=1\n' + FRAGMENT.read_text())
        install_shell_config(self.bashrc, REPO_DIR)
        updated = self.bashrc.read_text()
        self.assertIn('export ROS_LOCALHOST_ONLY=1', updated)
        self.assertNotIn('_robu_path=', updated)
        self.assertIn('tool_paths.sh', updated)

    def test_source_uses_home_variable_for_repository_inside_home(self) -> None:
        with patch('setup_tool_paths.Path.home', return_value=REPO_DIR.parent):
            install_shell_config(self.bashrc, REPO_DIR)
        relative = FRAGMENT.relative_to(REPO_DIR.parent)
        self.assertIn('source "$HOME"/' + shlex.quote(str(relative)), self.bashrc.read_text())

    def test_source_handles_external_repository_with_spaces_and_shell_characters(self) -> None:
        repo = self.root / 'repo with spaces $(false)'
        fragment = repo / 'config' / 'shell' / 'tool_paths.sh'
        fragment.parent.mkdir(parents=True)
        fragment.write_text('ROBU_TEST_CONFIG_LOADED=yes\n')
        install_shell_config(self.bashrc, repo)
        result = subprocess.run(
            [
                'bash',
                '-c',
                f'source {shlex.quote(str(self.bashrc))}; echo "$ROBU_TEST_CONFIG_LOADED"',
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), 'yes')

    def test_incomplete_block_and_missing_config_leave_profile_unchanged(self) -> None:
        original = '# >>> ROBU tool paths >>>\ncustom command\n'
        self.bashrc.write_text(original)
        with self.assertRaises(ValueError):
            install_shell_config(self.bashrc, REPO_DIR)
        with self.assertRaises(FileNotFoundError):
            install_shell_config(self.bashrc, self.root / 'missing')
        self.assertEqual(self.bashrc.read_text(), original)

    def test_stale_links_are_repaired_and_use_isolated_core(self) -> None:
        self.launchers.mkdir(parents=True)
        for name in ('pio', 'platformio', 'piodebuggdb'):
            self.create_command(name)
            (self.launchers / name).symlink_to(self.root / 'deleted snap' / name)
        self.assertTrue(link_platformio_commands(self.core, self.launchers, required=True))
        for name in ('pio', 'platformio', 'piodebuggdb'):
            link = self.launchers / name
            self.assertEqual(link.resolve(), self.core / name)
            previous = link.lstat().st_mtime_ns
            self.assertTrue(link_platformio_commands(self.core, self.launchers, required=True))
            self.assertEqual(link.lstat().st_mtime_ns, previous)
        result = subprocess.run([str(self.launchers / 'pio')], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(result.stdout.strip(), 'isolated core')

    def test_custom_user_launcher_is_preserved(self) -> None:
        for name in ('pio', 'platformio'):
            self.create_command(name)
        self.launchers.mkdir(parents=True)
        custom = self.launchers / 'pio'
        custom.write_text('# custom wrapper\n')
        self.assertTrue(link_platformio_commands(self.core, self.launchers, required=True))
        self.assertFalse(custom.is_symlink())
        self.assertEqual(custom.read_text(), '# custom wrapper\n')
        self.assertTrue((self.launchers / 'platformio').is_symlink())

    def test_core_can_be_absent_during_bootstrap_but_not_after_installation(self) -> None:
        self.assertTrue(link_platformio_commands(self.core, self.launchers, required=False))
        self.assertFalse(self.launchers.exists())
        self.create_command('pio')
        with contextlib.redirect_stderr(self.output):
            self.assertFalse(link_platformio_commands(self.core, self.launchers, required=True))
        self.assertTrue((self.launchers / 'pio').is_symlink())
        self.create_command('platformio')
        self.assertTrue(link_platformio_commands(self.core, self.launchers, required=True))

    def test_path_cleanup_is_idempotent_and_keeps_other_environments(self) -> None:
        home = Path.home()
        pio_bin = str(home / '.platformio/penv/bin')
        local_bin = str(home / '.local/bin')
        dev_bin = str(self.root / 'development/bin')
        for parts in (
            (pio_bin, '/usr/bin', '/bin'),
            ('/usr/bin', '/bin', pio_bin),
            (local_bin, pio_bin, dev_bin, local_bin, pio_bin, '/usr/bin', '/bin'),
        ):
            with self.subTest(parts=parts):
                body = (
                    f'PATH={shlex.quote(":".join(parts))}\n'
                    f'source {shlex.quote(str(FRAGMENT))}\n'
                    'first="$PATH"\n'
                    f'source {shlex.quote(str(FRAGMENT))}\n'
                    '[[ "$first" = "$PATH" ]] || exit 1\n'
                    'echo "$PATH"\ncommand -v python3\ncommand -v colcon\n'
                )
                result = subprocess.run(
                    ['bash', '-c', body],
                    env=os.environ.copy(),
                    text=True,
                    capture_output=True,
                    check=False,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                cleaned, python, colcon = result.stdout.splitlines()
                self.assertNotIn(pio_bin, cleaned.split(':'))
                self.assertEqual(cleaned.split(':').count(local_bin), 1)
                self.assertTrue(cleaned.startswith(local_bin + ':'))
                if dev_bin in parts:
                    self.assertIn(dev_bin, cleaned.split(':'))
                else:
                    self.assertEqual(python, '/usr/bin/python3')
                self.assertEqual(colcon, '/usr/bin/colcon')


if __name__ == '__main__':
    unittest.main()
