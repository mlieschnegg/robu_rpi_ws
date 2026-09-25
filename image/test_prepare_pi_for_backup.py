"""Check privilege boundaries without running cleanup or system commands."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).with_name('prepare_pi_for_backup.sh')


class PreparationChecks(unittest.TestCase):
    def run_mocked(self, *args, deny=False, github="none", token=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            home = root / 'home'
            home.mkdir()
            log = root / 'commands'
            stub = root / 'stub'
            stub.write_text('''#!/bin/bash
name=${0##*/}
printf '%s' "$name" >> "$TEST_LOG"
printf ' <%s>' "$@" >> "$TEST_LOG"
printf '\\n' >> "$TEST_LOG"
if [[ "$name" == sudo && "$1" == -v && "$TEST_DENY" == yes ]]; then exit 1; fi
if [[ "$name" == gh ]]; then
  if [[ "$*" == *--help* ]]; then
    if [[ "$TEST_GITHUB" != legacy* ]]; then echo '  --user string'; fi
    exit 0
  fi
  if [[ "$TEST_GITHUB" == legacy* && "$2" == status ]]; then
    if [[ "$*" == *--json* ]]; then echo 'unknown flag: --json' >&2; exit 1; fi
    if [[ "$TEST_GITHUB" == legacy_error ]]; then echo 'network error' >&2; exit 1; fi
    if [[ "$TEST_GITHUB" == legacy_unknown ]]; then echo 'unexpected output' >&2; exit 0; fi
    if [[ "$TEST_GITHUB" == legacy_none || -e "$TEST_LOG.loggedout" ]]; then
      echo 'You are not logged into any GitHub hosts. Run gh auth login to authenticate.' >&2
      exit 1
    fi
    echo github.com >&2
    if [[ "$TEST_GITHUB" == legacy_personal || "$TEST_GITHUB" == legacy_mixed ]]; then
      echo '  ✓ Logged in to github.com as mlieschnegg (/home/robu/.config/gh/hosts.yml)' >&2
    fi
    if [[ "$TEST_GITHUB" == legacy_school || "$TEST_GITHUB" == legacy_mixed ]]; then
      echo '  ✓ Logged in to github.com account techtitans-htlk (/home/robu/.config/gh/hosts.yml)' >&2
      echo '  - Active account: true' >&2
      echo '  - Token: ghp_************' >&2
    fi
    exit 0
  fi
  if [[ "$2" == status ]]; then
    [[ "$TEST_GITHUB" != status_fail ]] || exit 1
    if [[ "$TEST_GITHUB" == malformed ]]; then echo invalid; exit 0; fi
    if [[ "$TEST_GITHUB" != none && ! -e "$TEST_LOG.loggedout" ]]; then
      echo '{"hosts":{"github.com":[{"login":"student","active":true},{"login":"mlieschnegg","active":false}]}}'
    else
      echo '{"hosts":{"github.com":[{"login":"student","active":true}]}}'
    fi
  elif [[ "$2" == logout ]]; then
    [[ "$TEST_GITHUB" != logout_fail ]] || exit 1
    if [[ "$TEST_GITHUB" != retained ]]; then touch "$TEST_LOG.loggedout"; fi
  fi
fi
if [[ "$name" == git ]]; then cat >/dev/null; fi
''')
            stub.chmod(0o755)
            for name in ['sudo', 'rm', 'snap', 'apt', 'apt-get', 'dpkg',
                         'debsums', 'sync', 'sleep', 'shutdown', 'gh', 'git']:
                (root / name).symlink_to(stub)
            # Only normalize EUID when tests themselves run under root.
            script = root / 'script.sh'
            script.write_text(SCRIPT.read_text().replace(
                '[[ $EUID -eq 0 ]]', '[[ 1000 -eq 0 ]]'))
            env = dict(os.environ, HOME=str(home), PATH=f'{root}:/usr/bin:/bin',
                       TEST_LOG=str(log), TEST_DENY='yes' if deny else 'no',
                       TEST_GITHUB=github)
            env.pop('GH_TOKEN', None)
            env.pop('GITHUB_TOKEN', None)
            if token:
                env['GH_TOKEN'] = 'test-placeholder'
            result = subprocess.run(['bash', str(script), *args], env=env,
                                    text=True, capture_output=True)
            return result, log.read_text() if log.exists() else '', str(home)

    def test_user_cleanup_and_privileged_system_actions(self):
        result, log, home = self.run_mocked('--no-shutdown')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(log.startswith('sudo <-v>\n'))
        self.assertIn(f'rm <-f> <--> <{home}/.bash_history>', log)
        self.assertIn(f'<{home}/Downloads/*>', log)
        self.assertIn('sudo <env> <DEBIAN_FRONTEND=noninteractive> <apt> <-y> <upgrade>', log)
        for tool in ['apt', 'apt-get', 'dpkg', 'debsums', 'snap']:
            self.assertIn(f'sudo <{tool}>', log)
            self.assertFalse(any(line.startswith(tool + ' ') for line in log.splitlines()))
        self.assertNotIn('shutdown', log)
        self.assertNotIn('sudo <rm>', log)

    def test_shutdown_uses_sudo(self):
        result, log, _ = self.run_mocked()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('sudo <shutdown> <-h> <now>', log)

    def test_denied_sudo_stops_before_cleanup(self):
        result, log, _ = self.run_mocked(deny=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(log, 'sudo <-v>\n')

    def test_invalid_option_has_no_side_effects(self):
        result, log, _ = self.run_mocked('--typo')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(log, '')

    def test_github_inactive_account_logged_out(self):
        result, log, _ = self.run_mocked('--no-shutdown', github='present')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('gh <auth> <logout> <--hostname> <github.com> <--user> <mlieschnegg>', log)
        self.assertEqual(log.count('gh <auth> <status>'), 2)
        self.assertIn('git <credential> <reject>', log)

    def test_github_absent_not_logged_out(self):
        result, log, _ = self.run_mocked('--no-shutdown')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('<logout>', log)

    def test_github_failures_stop_preparation(self):
        for mode in ['status_fail', 'malformed', 'logout_fail', 'retained']:
            with self.subTest(mode=mode):
                result, log, _ = self.run_mocked(github=mode)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn('rm <', log)
                self.assertNotIn('sudo <apt>', log)
                self.assertNotIn('shutdown', log)

    def test_environment_token_blocks_preparation(self):
        result, log, _ = self.run_mocked(github='present', token=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('<logout>', log)
        self.assertNotIn('rm <', log)
        self.assertNotIn('test-placeholder', result.stdout + result.stderr + log)

    def test_legacy_school_account_kept(self):
        result, log, _ = self.run_mocked('--no-shutdown', github='legacy_school')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('<logout>', log)

    def test_legacy_personal_account_logged_out(self):
        result, log, _ = self.run_mocked('--no-shutdown', github='legacy_personal')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('gh <auth> <logout> <--hostname> <github.com>\n', log)
        self.assertNotIn('<--user>', log)

    def test_legacy_no_account(self):
        result, log, _ = self.run_mocked('--no-shutdown', github='legacy_none')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('<logout>', log)

    def test_legacy_unsafe_status_stops(self):
        for mode in ['legacy_error', 'legacy_unknown', 'legacy_mixed']:
            with self.subTest(mode=mode):
                result, log, _ = self.run_mocked(github=mode)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn('rm <', log)
                self.assertNotIn('<logout> <--hostname>', log)


if __name__ == '__main__':
    unittest.main()
