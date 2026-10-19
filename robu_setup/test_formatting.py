"""Verify shared defaults, portable copies and preservation of existing project files."""

import io
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

from setup_formatting import install_formatting_configs


class FormattingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.templates = self.root / 'repo' / 'config' / 'formatting'
        self.templates.mkdir(parents=True)
        (self.templates / '.clang-format').write_text('BasedOnStyle: LLVM\nIndentWidth: 4\n')
        (self.templates / 'pyproject.toml').write_text('[tool.ruff]\nline-length = 99\n')
        self.work = self.root / 'work'

    def install(self, copy_files: bool = False) -> bool:
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return install_formatting_configs(self.work, self.templates, copy_files)

    def test_shared_links_are_relative_and_second_run_keeps_them(self) -> None:
        self.assertTrue(self.install())
        timestamps = {}
        for name in ('.clang-format', 'pyproject.toml'):
            target = self.work / name
            self.assertTrue(target.is_symlink())
            self.assertFalse(target.readlink().is_absolute())
            self.assertEqual(target.resolve(), self.templates / name)
            timestamps[name] = target.lstat().st_mtime_ns
        self.assertTrue(self.install())
        self.assertEqual(
            timestamps, {name: (self.work / name).lstat().st_mtime_ns for name in timestamps}
        )

    def test_existing_project_configs_are_preserved(self) -> None:
        self.work.mkdir()
        originals = {
            '.clang-format': 'IndentWidth: 2\n',
            'pyproject.toml': '[project]\nname = "my_project"\n',
        }
        for name, content in originals.items():
            (self.work / name).write_text(content)
        for copy_files in (False, True):
            self.assertTrue(self.install(copy_files))
            for name, content in originals.items():
                self.assertEqual((self.work / name).read_text(), content)

    def test_project_copies_are_independent_of_shared_templates(self) -> None:
        self.assertTrue(self.install(copy_files=True))
        for name in ('.clang-format', 'pyproject.toml'):
            self.assertFalse((self.work / name).is_symlink())
            self.assertEqual((self.work / name).read_bytes(), (self.templates / name).read_bytes())
        (self.templates / 'pyproject.toml').write_text('[tool.ruff]\nline-length = 80\n')
        self.assertIn('line-length = 99', (self.work / 'pyproject.toml').read_text())
        self.assertTrue(self.install(copy_files=True))
        self.assertIn('line-length = 99', (self.work / 'pyproject.toml').read_text())

    def test_missing_first_template_does_not_block_second_config(self) -> None:
        (self.templates / '.clang-format').unlink()
        self.assertFalse(self.install())
        self.assertEqual(
            (self.work / 'pyproject.toml').resolve(), self.templates / 'pyproject.toml'
        )
        self.assertFalse((self.work / '.clang-format').exists())

    def test_existing_dangling_symlink_is_preserved(self) -> None:
        self.work.mkdir()
        target = self.work / '.clang-format'
        target.symlink_to('custom-but-currently-unavailable')
        self.assertTrue(self.install())
        self.assertEqual(target.readlink(), Path('custom-but-currently-unavailable'))


if __name__ == '__main__':
    unittest.main()
