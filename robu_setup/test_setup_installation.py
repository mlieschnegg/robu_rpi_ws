"""Test setup failures with fake installers; never run apt, sudo or real pip."""

from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


SETUP_DIR = Path(__file__).resolve().parent


class InstallationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def shell(self, body):
        return subprocess.run(['bash', '-c', body], text=True, capture_output=True, check=False)

    def source(self, filename='setup_common.sh'):
        return f'source {shlex.quote(str(SETUP_DIR / filename))}\n'

    def test_failure_inside_function_does_not_stop_remaining_commands(self):
        result = self.shell(self.source() + '''
            failing_step() { /bin/false; echo "remaining command executed"; }
            failing_step
            echo "next step executed"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertIn('remaining command executed', result.stdout)
        self.assertIn('next step executed', result.stdout)
        self.assertIn('exit 1', result.stderr)

    def test_success_has_successful_exit_and_empty_error_summary(self):
        result = self.shell(self.source() + '''
            echo "all work completed"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 0)
        self.assertIn('completed successfully', result.stdout)
        self.assertEqual(result.stderr, '')

    def test_one_failed_extension_does_not_block_the_rest(self):
        result = self.shell(self.source() + f'''
            SCRIPT_DIR={shlex.quote(str(SETUP_DIR))}
            code() {{
                echo "extension: $2"
                if [ "$2" = "charliermarsh.ruff" ]; then return 9; fi
                return 0
            }}
            install_vscode_extensions
            echo "after extensions"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        extensions = [line.split(': ', 1)[1] for line in result.stdout.splitlines()
                      if line.startswith('extension: ')]
        self.assertIn('charliermarsh.ruff', extensions)
        self.assertIn('pdconsec.vscode-print', extensions)
        self.assertEqual(len(extensions), len(set(extensions)))
        self.assertNotIn('Ranch-Hand-Robotics.rde-pack', extensions)
        self.assertIn('after extensions', result.stdout)

    def test_missing_venv_cannot_install_into_system_python(self):
        result = self.shell(self.source() + f'''
            VENV_DIR={shlex.quote(str(self.root / 'missing venv'))}
            python() {{ echo "SYSTEM PYTHON CALLED"; }}
            python3() {{ echo "SYSTEM PYTHON CALLED"; }}
            install_python_dev_tools
            echo "later step"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertNotIn('SYSTEM PYTHON CALLED', result.stdout)
        self.assertIn('later step', result.stdout)

    def test_exact_ruff_requirement_is_installed_even_when_old_version_exists(self):
        venv = self.root / 'venv with spaces'
        (venv / 'bin').mkdir(parents=True)
        interpreter = venv / 'bin' / 'python'
        interpreter.write_text('#!/bin/bash\nprintf "pip:"\nprintf " <%s>" "$@"\nprintf "\\n"\nexit 6\n')
        interpreter.chmod(0o755)
        ruff = venv / 'bin' / 'ruff'
        ruff.write_text('#!/bin/bash\necho "ruff 0.16.10"\n')
        ruff.chmod(0o755)
        result = self.shell(self.source() + f'''
            VENV_DIR={shlex.quote(str(venv))}
            SCRIPT_DIR={shlex.quote(str(SETUP_DIR))}
            is_raspberry_pi() {{ return 1; }}
            install_python_dev_tools
            echo "after tools"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertIn('<ruff==0.17.0>', result.stdout)
        self.assertIn('<clang-format==19.1.7>', result.stdout)
        self.assertIn('after tools', result.stdout)

    def test_pi_avoids_building_unsupported_clang_format_package(self):
        venv = self.root / 'pi-venv'
        (venv / 'bin').mkdir(parents=True)
        for name, body in (('python', 'printf "%s\\n" "$@"'), ('ruff', 'echo "ruff 0.17.0"')):
            executable = venv / 'bin' / name
            executable.write_text('#!/bin/bash\n' + body + '\n')
            executable.chmod(0o755)
        result = self.shell(self.source() + f'''
            VENV_DIR={shlex.quote(str(venv))}
            SCRIPT_DIR={shlex.quote(str(SETUP_DIR))}
            is_raspberry_pi() {{ return 0; }}
            install_python_dev_tools
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 0)
        self.assertIn('ruff==0.17.0', result.stdout)
        self.assertNotIn('clang-format', result.stdout)

    def test_ubuntu_server_continues_after_child_exits_and_uses_isolated_shell(self):
        children = self.root / 'children'
        children.mkdir()
        (children / 'robu_setup.sh').write_text('exit 7\n')
        (children / 'rpi_setup.sh').write_text('echo "following child executed"\n')
        result = self.shell(self.source('ubuntu_server_setup.sh') + f'''
            SCRIPT_DIR={shlex.quote(str(children))}
            run_sub_setup_scripts
            echo "parent continued"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertIn('following child executed', result.stdout)
        self.assertIn('parent continued', result.stdout)

    def test_optional_venv_packages_never_fall_back_to_system_python(self):
        for filename in ('robu_setup.sh', 'rpi_setup.sh'):
            with self.subTest(script=filename):
                result = self.shell(self.source(filename) + f'''
                    VENV_DIR={shlex.quote(str(self.root / 'missing venv'))}
                    python() {{ echo "SYSTEM PYTHON CALLED"; }}
                    install_venv_python_pkg_if_missing pytest
                    echo "later step"
                    if robu_setup_report; then exit 0; else exit 1; fi
                ''')
                self.assertEqual(result.returncode, 1)
                self.assertNotIn('SYSTEM PYTHON CALLED', result.stdout)
                self.assertIn('later step', result.stdout)

    def test_unavailable_pishrink_directory_does_not_exit_setup(self):
        result = self.shell(self.source('robu_setup.sh') + f'''
            WORK_DIR={shlex.quote(str(self.root / 'missing work'))}
            wget() {{ echo "DOWNLOAD SHOULD NOT RUN"; }}
            install_pishrink
            echo "later step"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertNotIn('DOWNLOAD SHOULD NOT RUN', result.stdout)
        self.assertIn('later step', result.stdout)

    def test_robu_main_runs_later_steps_after_failed_base_installation(self):
        functions = ('setup_tool_paths', 'setup_keyboard', 'install_exaloop', 'setup_python_venv',
                     'setup_venv_auto_activate', 'install_python_dev_tools',
                     'setup_formatting_configs',
                     'install_python_packages', 'install_pc_tools', 'run_sub_setup_scripts',
                     'setup_shell_and_permissions', 'apply_runtime_device_permissions',
                     'print_runtime_checks')
        stubs = '\n'.join(f'{name}() {{ echo "step: {name}"; }}' for name in functions)
        result = self.shell(self.source('robu_setup.sh') + f'''
            WORK_DIR={shlex.quote(str(self.root / 'work'))}
            {stubs}
            is_raspberry_pi() {{ return 1; }}
            install_base_packages() {{ /bin/false; }}
            main
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertIn('step: install_pc_tools', result.stdout)
        self.assertIn('step: setup_shell_and_permissions', result.stdout)

    def test_snippet_linker_failure_keeps_pc_setup_and_parent_running(self):
        children = self.root / 'helpers'
        children.mkdir()
        (children / 'link_vsc_snippets.sh').write_text('set -e\nexit 5\n')
        result = self.shell(self.source('robu_setup.sh') + f'''
            SCRIPT_DIR={shlex.quote(str(children))}
            SYS_PYTHON=fake_python
            fake_python() {{ return 0; }}
            sudo() {{ return 0; }}
            mkdir() {{ echo "desktop step executed"; }}
            cp() {{ return 0; }}
            is_raspberry_pi() {{ return 1; }}
            install_vscode_extensions() {{ return 0; }}
            install_pc_tools
            echo "parent continued"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertIn('desktop step executed', result.stdout)
        self.assertIn('parent continued', result.stdout)

    def test_working_platformio_is_reused_on_pc_and_pi(self):
        core = self.root / 'core with spaces' / 'bin'
        core.mkdir(parents=True)
        for name in ('pio', 'platformio'):
            command = core / name
            command.write_text('#!/bin/bash\necho "PlatformIO Core test"\n')
            command.chmod(0o755)
        for pi_status in (0, 1):
            with self.subTest(pi_status=pi_status):
                result = self.shell(self.source('microros_setup.sh') + f'''
                    PLATFORMIO_BIN={shlex.quote(str(core))}
                    is_raspberry_pi() {{ return {pi_status}; }}
                    curl() {{ echo "UNNECESSARY DOWNLOAD"; return 7; }}
                    setup_tool_paths() {{ printf 'links:'; printf ' <%s>' "$@"; echo; }}
                    install_platformio
                    if robu_setup_report; then exit 0; else exit 1; fi
                ''')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertNotIn('UNNECESSARY DOWNLOAD', result.stdout)
                self.assertIn('<--link-platformio>', result.stdout)
                self.assertIn('PlatformIO Core test', result.stdout)

    def test_failed_platformio_download_skips_execution_and_keeps_next_step(self):
        result = self.shell(self.source('microros_setup.sh') + f'''
            PLATFORMIO_BIN={shlex.quote(str(self.root / 'missing core'))}
            TMPDIR={shlex.quote(str(self.root))}
            curl() {{ return 22; }}
            SYS_PYTHON=fake_python
            fake_python() {{ echo "INCOMPLETE INSTALLER EXECUTED"; }}
            setup_tool_paths() {{ echo "LINKS SHOULD NOT RUN"; }}
            install_platformio
            echo "next step"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertIn('exit 22', result.stderr)
        self.assertIn('next step', result.stdout)
        self.assertNotIn('INSTALLER EXECUTED', result.stdout)
        self.assertNotIn('LINKS SHOULD NOT RUN', result.stdout)
        self.assertEqual(list(self.root.glob('robu-platformio.*.py')), [])

    def test_platformio_installer_uses_explicit_system_python(self):
        core = self.root / 'core' / 'bin'
        fixture = self.root / 'fake-installer.py'
        fixture.write_text(f'''
import sys
from pathlib import Path
print("installer interpreter:", sys.executable)
core = Path({str(core)!r})
core.mkdir(parents=True)
for name in ("pio", "platformio"):
    executable = core / name
    executable.write_text("#!/bin/bash\\necho 'PlatformIO Core test'\\n")
    executable.chmod(0o755)
''')
        result = self.shell(self.source('microros_setup.sh') + f'''
            PLATFORMIO_BIN={shlex.quote(str(core))}
            TMPDIR={shlex.quote(str(self.root))}
            curl() {{ cp {shlex.quote(str(fixture))} "$3"; }}
            python3() {{ echo "PATH PYTHON CALLED"; return 9; }}
            setup_tool_paths() {{ echo "links installed"; }}
            install_platformio
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('installer interpreter: /usr/bin/python3', result.stdout)
        self.assertIn('links installed', result.stdout)
        self.assertNotIn('PATH PYTHON CALLED', result.stdout)
        self.assertEqual(list(self.root.glob('robu-platformio.*.py')), [])

    def test_failed_platformio_installer_does_not_block_following_steps(self):
        result = self.shell(self.source('microros_setup.sh') + f'''
            PLATFORMIO_BIN={shlex.quote(str(self.root / 'missing core'))}
            TMPDIR={shlex.quote(str(self.root))}
            curl() {{ return 0; }}
            SYS_PYTHON=fake_python
            fake_python() {{ return 6; }}
            setup_tool_paths() {{ echo "LINKS SHOULD NOT RUN"; }}
            install_platformio
            echo "next step"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertIn('Core installer failed (exit 6)', result.stderr)
        self.assertIn('next step', result.stdout)
        self.assertNotIn('LINKS SHOULD NOT RUN', result.stdout)
        self.assertEqual(list(self.root.glob('robu-platformio.*.py')), [])

    def test_platformio_setup_precedes_ros_check(self):
        result = self.shell(self.source('microros_setup.sh') + f'''
            WORK_DIR={shlex.quote(str(self.root / 'work'))}
            setup_tool_paths() {{ echo "tool paths"; }}
            install_base_packages() {{ return 0; }}
            install_platformio() {{ echo "PlatformIO setup"; }}
            install_teensy_rules() {{ return 0; }}
            install_raspberry_pi_tools() {{ return 0; }}
            require_ros() {{ echo "ROS unavailable"; exit 1; }}
            main
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertLess(result.stdout.index('PlatformIO setup'), result.stdout.index('ROS unavailable'))

    def test_failed_pi_loader_clone_keeps_other_setup_steps_running(self):
        result = self.shell(self.source('microros_setup.sh') + f'''
            WORK_DIR={shlex.quote(str(self.root))}
            is_raspberry_pi() {{ return 0; }}
            append_once() {{ return 0; }}
            git() {{ return 7; }}
            make() {{ echo "MAKE SHOULD NOT RUN"; }}
            sudo() {{ echo "INSTALL SHOULD NOT RUN"; }}
            install_raspberry_pi_tools
            echo "next step"
            if robu_setup_report; then exit 0; else exit 1; fi
        ''')
        self.assertEqual(result.returncode, 1)
        self.assertIn('next step', result.stdout)
        self.assertNotIn('SHOULD NOT RUN', result.stdout)


if __name__ == '__main__':
    unittest.main()
