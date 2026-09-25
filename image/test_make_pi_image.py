"""Read-only regression checks; no root, loop devices or SD cards required."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).with_name('make_pi_image.sh').read_text()
CHECK = SCRIPT.split("python3 -c '\n", 1)[1].split("\n' \"$file\"", 1)[0]


class ImageChecks(unittest.TestCase):
    def check_layout(self, partitions, label='dos', size=4096):
        with tempfile.NamedTemporaryFile() as image:
            image.truncate(size)
            return subprocess.run(
                ['python3', '-c', CHECK, image.name],
                input=json.dumps({'partitiontable': {
                    'label': label, 'sectorsize': 512,
                    'partitions': partitions}}),
                text=True, capture_output=True).returncode

    def test_valid(self):
        self.assertEqual(self.check_layout([
            {'start': 1, 'size': 2}, {'start': 3, 'size': 5}]), 0)

    def test_truncated(self):
        self.assertNotEqual(self.check_layout([
            {'start': 1, 'size': 2}, {'start': 3, 'size': 6}]), 0)

    def test_overlap(self):
        self.assertNotEqual(self.check_layout([
            {'start': 1, 'size': 3}, {'start': 3, 'size': 2}]), 0)

    def test_unsupported(self):
        self.assertNotEqual(self.check_layout([
            {'start': 1, 'size': 2}, {'start': 3, 'size': 5}], 'gpt'), 0)
        self.assertNotEqual(self.check_layout([{'start': 1, 'size': 7}]), 0)

    def test_compressed_readback(self):
        import gzip
        with tempfile.TemporaryDirectory() as directory:
            image = Path(directory) / 'image.gz'
            card = Path(directory) / 'card'
            payload = b'boot and root data' * 4096
            image.write_bytes(gzip.compress(payload))
            command = ['bash', '-o', 'pipefail', '-c',
                       'gzip -dc -- "$1" | cmp -n "$3" - "$2"',
                       'test', str(image), str(card), str(len(payload))]
            for data, ok in [(payload + b'extra capacity', True),
                             (b'X' + payload[1:], False),
                             (payload[:-1], False)]:
                card.write_bytes(data)
                result = subprocess.run(command, capture_output=True)
                self.assertEqual(result.returncode == 0, ok)
            image.write_bytes(image.read_bytes()[:-4])
            card.write_bytes(payload)
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)


class ResumeChecks(unittest.TestCase):
    """Exercise the script flow with disk operations replaced by file fixtures."""
    def setUp(self):
        import os
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.source = self.directory / 'source'
        self.payload = b'original card contents' * 100
        self.source.write_bytes(self.payload)
        self.out = self.directory / 'backup.img'
        self.log = self.directory / 'dd.log'
        mocks = r"""
check_device() { :; }
check_image() { [[ $(head -c 7 -- "$1") != CORRUPT ]] || die 'Corrupt image'; }
blockdev() { if [[ $1 == --getsize64 ]]; then stat -c %s "$2"; fi; }
df() {
  local count=0 free
  if [[ -f "$TEST_DD_LOG.df" ]]; then read -r count < "$TEST_DD_LOG.df"; fi
  count=$((count + 1))
  printf '%s\n' "$count" > "$TEST_DD_LOG.df"
  case $count in
    1) free=${TEST_SPACE_1:-999999999999};;
    2) free=${TEST_SPACE_2:-999999999999};;
    *) free=${TEST_SPACE_3:-999999999999};;
  esac
  printf 'Avail\n%s\n' "$free"
}
dd() {
  local arg writing=false
  local args=()
  for arg in "$@"; do
    [[ "$arg" != of=* || "$arg" == of=/dev/null ]] || writing=true
    arg=${arg/iflag=direct,/iflag=}
    args+=("$arg")
  done
  if $writing; then
    echo called >> "$TEST_DD_LOG"
    [[ ${TEST_MODE:-} != dd_fail ]] || return 1
  fi
  command dd "${args[@]}"
}

pishrink.sh() {
  case ${TEST_MODE:-} in
    fail) printf CORRUPT > "$1"; return 1;;
    corrupt) printf CORRUPT > "$1";;
    *) truncate -s 100 "$1";;
  esac
}
"""
        # Keep the production workflow; stub only device access and dependencies.
        script = SCRIPT.replace('[[ $EUID == 0 ]]', '[[ 0 == 0 ]]').replace('CHUNK_BYTES=1073741824', 'CHUNK_BYTES=4096')
        start = script.index('for tool in ')
        end = script.index('\ndone', start) + len('\ndone')
        script = script[:start] + script[end:]
        script = script.replace('OUT=$(realpath -m "$OUT")',
                                mocks + '\nOUT=$(realpath -m "$OUT")')
        self.script = self.directory / 'test-script.sh'
        self.script.write_text(script)
        self.env = dict(os.environ, TEST_DD_LOG=str(self.log))

    def run_script(self, mode, resume=False, existing=False):
        Path(str(self.log) + '.df').unlink(missing_ok=True)
        args = ['--resume'] if resume else ['-d', str(self.source), '--shrink']
        if existing:
            args.append('--check-existing')
        return subprocess.run(
            ['bash', str(self.script), *args, '-o', str(self.out)],
            env=dict(self.env, TEST_MODE=mode), text=True, capture_output=True)

    def check_retry(self, mode):
        result = self.run_script(mode)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.out.read_bytes(), self.payload)
        self.assertTrue(Path(str(self.out) + '.sha256').exists())
        self.assertFalse(Path(str(self.out) + '.gz').exists())
        self.assertIn('--resume', result.stderr)
        self.assertFalse(list(self.directory.glob('*.shrink.*')))
        self.source.unlink()  # Resume must work without the original card.
        result = self.run_script('ok', resume=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.log.read_text(), 'called\n')
        self.assertEqual(self.out.read_bytes(), self.payload)
        import gzip
        self.assertEqual(gzip.decompress(Path(str(self.out) + '.gz').read_bytes()),
                         self.payload[:100])
        for name in ['backup.img.sha256', 'backup.img.gz.sha256']:
            self.assertEqual(subprocess.run(
                ['sha256sum', '-c', name], cwd=self.directory,
                capture_output=True).returncode, 0)

    def test_failed_shrink_can_resume(self):
        self.check_retry('fail')

    def test_corrupt_shrink_can_resume(self):
        self.check_retry('corrupt')

    def test_changed_raw_image_rejected(self):
        self.run_script('fail')
        self.out.write_bytes(b'changed' + self.payload[7:])
        self.assertNotEqual(self.run_script('ok', resume=True).returncode, 0)
        self.assertEqual(self.log.read_text(), 'called\n')
        self.assertFalse(Path(str(self.out) + '.gz').exists())

    def test_dd_failure_not_resumable(self):
        self.assertNotEqual(self.run_script('dd_fail').returncode, 0)
        self.assertFalse(self.out.exists())
        self.assertNotEqual(self.run_script('ok', resume=True).returncode, 0)

    def test_existing_result_not_overwritten(self):
        self.assertEqual(self.run_script('ok').returncode, 0)
        packed = Path(str(self.out) + '.gz').read_bytes()
        self.assertNotEqual(self.run_script('ok', resume=True).returncode, 0)
        self.assertEqual(Path(str(self.out) + '.gz').read_bytes(), packed)

    def test_space_shortage_before_dd(self):
        self.env['TEST_SPACE_1'] = str(1073741824 + len(self.payload) - 1)
        result = self.run_script('ok')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Rohkopie', result.stderr)
        self.assertFalse(self.log.exists())
        self.assertFalse(self.out.exists())

    def test_later_space_shortages_preserve_raw_and_resume(self):
        for phase, stage in [('Shrink-Arbeitskopie', 2), ('Komprimierung', 3)]:
            with self.subTest(phase=phase):
                # Each scenario uses its own fixture through a separate instance.
                fixture = ResumeChecks()
                fixture.setUp()
                try:
                    fixture.env[f'TEST_SPACE_{stage}'] = '0'
                    result = fixture.run_script('ok')
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(phase, result.stderr)
                    self.assertIn('--resume', result.stderr)
                    self.assertEqual(fixture.out.read_bytes(), fixture.payload)
                    self.assertTrue(Path(str(fixture.out) + '.sha256').exists())
                    self.assertFalse(list(fixture.directory.glob('*.shrink.*')))
                    self.assertFalse(Path(str(fixture.out) + '.gz').exists())
                    fixture.env.pop(f'TEST_SPACE_{stage}')
                    fixture.source.unlink()
                    result = fixture.run_script('ok', resume=True)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(fixture.log.read_text(), 'called\n')
                finally:
                    fixture.doCleanups()

    def test_space_checked_per_stage_using_shrunk_size(self):
        reserve = 1073741824
        # Enough for each next write, but not the previous three-copy estimate.
        self.env.update(TEST_SPACE_1=str(reserve + len(self.payload)),
                        TEST_SPACE_2=str(reserve + len(self.payload)),
                        TEST_SPACE_3=str(reserve + 100 + 1 + 1048576))
        result = self.run_script('ok')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(Path(str(self.log) + '.df').read_text(), '3\n')

    def test_existing_rescue_no_copy_and_can_shrink(self):
        self.out.write_bytes(self.payload)
        result = self.run_script('ok', existing=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(self.log.exists())
        self.assertEqual(self.out.read_bytes(), self.payload)
        self.assertTrue(Path(str(self.out) + '.gz').exists())

    def test_existing_mismatch_preserved_no_metadata_or_shrink(self):
        rescued = b'X' + self.payload[1:]
        self.out.write_bytes(rescued)
        result = self.run_script('ok', existing=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Abschnitt 1', result.stderr)
        self.assertEqual(self.out.read_bytes(), rescued)
        self.assertFalse(self.log.exists())
        self.assertFalse(Path(str(self.out) + '.sha256').exists())
        self.assertFalse(Path(str(self.out) + '.gz').exists())

    def test_existing_short_image_rejected(self):
        self.out.write_bytes(self.payload[:-1])
        result = self.run_script('ok', existing=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('gleich groß', result.stderr)
        self.assertFalse(self.log.exists())

    def test_multiple_chunks_and_short_tail(self):
        self.payload = bytes(range(256)) * 35 + b'last bytes'
        self.source.write_bytes(self.payload)
        result = self.run_script('ok')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.out.read_bytes(), self.payload)
        self.assertEqual(self.log.read_text(), 'called\n' * 3)
        self.assertIn('Abschnitt 3 stimmt überein', result.stdout)

    def test_existing_mismatch_in_second_chunk(self):
        self.payload = b'A' * 9000
        self.source.write_bytes(self.payload)
        rescued = self.payload[:5000] + b'X' + self.payload[5001:]
        self.out.write_bytes(rescued)
        result = self.run_script('ok', existing=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Abschnitt 1 stimmt überein', result.stdout)
        self.assertIn('Abschnitt 2 (ab Byte 4096)', result.stderr)
        self.assertNotIn('Abschnitt 3:', result.stdout)
        self.assertEqual(self.out.read_bytes(), rescued)


if __name__ == '__main__':
    unittest.main()
