"""Run with: python3 -m unittest discover -s robu_setup -p 'test_vscode_settings.py'."""

import json
from pathlib import Path
import tempfile
import unittest

from setup_vscode_settings import configure


class SettingsTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.settings = self.root / "Code" / "User" / "settings.json"
        self.release = self.root / "os-release"
        self.release.write_text('VERSION_ID="24.04"\n', encoding="utf-8")

    def run_setup(self):
        configure(self.settings, self.release)
        return json.loads(self.settings.read_text(encoding="utf-8"))

    def test_fresh_image_and_second_run(self):
        settings = self.run_setup()
        self.assertEqual(settings["C_Cpp.intelliSenseEngine"], "disabled")
        self.assertTrue(settings["python.analysis.indexing"])
        self.assertIn("/opt/ros/jazzy/local/lib/python3.12/dist-packages",
                      settings["python.analysis.extraPaths"])
        timestamp = self.settings.stat().st_mtime_ns
        self.assertEqual(settings, self.run_setup())
        self.assertEqual(timestamp, self.settings.stat().st_mtime_ns)
        self.assertFalse(list(self.settings.parent.glob("*.bak")))

    def test_jsonc_preserves_preferences_and_one_original_backup(self):
        self.settings.parent.mkdir(parents=True)
        original = '''{
            // Keep this original comment in the backup.
            "editor.fontSize": 18,
            "python.analysis.extraPaths": ["/my/interfaces"],
            "python.analysis.languageServerMode": "light",
        }'''
        self.settings.write_text(original, encoding="utf-8")
        settings = self.run_setup()
        self.assertEqual(settings["editor.fontSize"], 18)
        self.assertIn("/my/interfaces", settings["python.analysis.extraPaths"])
        self.assertEqual(settings["python.analysis.languageServerMode"], "default")
        self.run_setup()
        backups = list(self.settings.parent.glob("*.bak"))
        self.assertEqual(len(backups), 1)
        self.assertEqual(backups[0].read_text(encoding="utf-8"), original)

    def test_invalid_input_is_not_overwritten(self):
        self.settings.parent.mkdir(parents=True)
        for invalid in ('{"unfinished":', '[]', '{"python.analysis.extraPaths": 42}'):
            with self.subTest(invalid=invalid):
                self.settings.write_text(invalid, encoding="utf-8")
                with self.assertRaises(ValueError):
                    self.run_setup()
                self.assertEqual(self.settings.read_text(encoding="utf-8"), invalid)
                self.assertFalse(list(self.settings.parent.glob("*.bak")))

    def test_humble_paths(self):
        self.release.write_text('VERSION_ID="22.04"\n', encoding="utf-8")
        settings = self.run_setup()
        self.assertIn("/opt/ros/humble/local/lib/python3.10/dist-packages",
                      settings["python.analysis.extraPaths"])
        self.assertFalse(any("jazzy" in p for p in settings["python.analysis.extraPaths"]))

    def test_remote_disables_language_server_without_changing_desktop(self):
        desktop = self.run_setup()
        desktop_bytes = self.settings.read_bytes()
        remote = self.root / ".vscode-server" / "data" / "Machine" / "settings.json"
        configure(remote, self.release, remote=True)
        settings = json.loads(remote.read_text(encoding="utf-8"))
        self.assertEqual(settings["python.languageServer"], "None")
        self.assertEqual(settings["C_Cpp.intelliSenseEngine"], "disabled")
        self.assertNotIn("python.analysis.indexing", settings)
        self.assertNotIn("python.languageServer", desktop)
        self.assertEqual(self.settings.read_bytes(), desktop_bytes)
        timestamp = remote.stat().st_mtime_ns
        configure(remote, self.release, remote=True)
        self.assertEqual(remote.stat().st_mtime_ns, timestamp)

    def test_remote_preserves_existing_jsonc_and_backup(self):
        self.settings.parent.mkdir(parents=True)
        original = '{ // Remote preferences\n"python.languageServer": "Pylance", "editor.fontSize": 19,}'
        self.settings.write_text(original, encoding="utf-8")
        configure(self.settings, self.release, remote=True)
        settings = json.loads(self.settings.read_text(encoding="utf-8"))
        self.assertEqual(settings["python.languageServer"], "None")
        self.assertEqual(settings["editor.fontSize"], 19)
        self.assertEqual(self.settings.with_name("settings.json.pre-robu.bak").read_text(), original)


if __name__ == "__main__":
    unittest.main()
