#!/usr/bin/env python3
"""Apply Python-first VS Code user settings. Run as the desktop user, not root."""

import json
import os
from pathlib import Path
import shutil
import tempfile

import json5


def configure(settings_path, os_release_path="/etc/os-release"):
    settings_path = Path(settings_path)
    original = settings_path.read_text(encoding="utf-8-sig") if settings_path.exists() else None
    settings = json5.loads(original) if original and original.strip() else {}
    if not isinstance(settings, dict):
        raise ValueError("VS Code settings must be a JSON object; file left unchanged")

    # Disable both semantic C++ analysis and the Tag Parser for Python lessons.
    # Keep the extension installed for PlatformIO/debugging and opt-in C++ work.
    settings.update({
        "C_Cpp.intelliSenseEngine": "disabled",
        "C_Cpp.intelliSenseCacheSize": 0,
        "C_Cpp.default.browse.limitSymbolsToIncludedHeaders": True,
        "python.analysis.languageServerMode": "default",
        "python.analysis.indexing": True,
        "python.analysis.diagnosticMode": "openFilesOnly",
    })

    # Match ros_setup.sh without requiring ROS to be installed already.
    # Preserve additional paths, including paths to custom interface packages.
    os_release = Path(os_release_path)
    release = os_release.read_text() if os_release.exists() else ""
    ros_paths = []
    if 'VERSION_ID="24.04"' in release:
        ros_paths = ["/opt/ros/jazzy/lib/python3.12/site-packages",
                     "/opt/ros/jazzy/local/lib/python3.12/dist-packages"]
    elif 'VERSION_ID="22.04"' in release:
        ros_paths = ["/opt/ros/humble/lib/python3.10/site-packages",
                     "/opt/ros/humble/local/lib/python3.10/dist-packages"]
    if ros_paths:
        paths = settings.setdefault("python.analysis.extraPaths", [])
        if not isinstance(paths, list):
            raise ValueError("python.analysis.extraPaths must be an array; file left unchanged")
        for path in ros_paths:
            if path not in paths:
                paths.append(path)

    updated = json.dumps(settings, indent=4, ensure_ascii=False) + "\n"
    if original == updated:
        print(f"[ROBU] VS Code settings already configured: {settings_path}")
        return

    settings_path.parent.mkdir(parents=True, exist_ok=True)
    # One fixed backup keeps the original comments without growing on each run.
    backup = settings_path.with_name(settings_path.name + ".pre-robu.bak")
    if original is not None and not backup.exists():
        shutil.copy2(settings_path, backup)

    # Atomic replacement avoids a partial settings file if writing is interrupted.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         dir=settings_path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(updated)
        if settings_path.exists():
            shutil.copymode(settings_path, temporary)
        temporary.replace(settings_path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    print(f"[ROBU] Configured VS Code user settings: {settings_path}")


if __name__ == "__main__":
    if os.geteuid() == 0:
        raise SystemExit("Run this helper as the desktop user, without sudo.")
    config_dir = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    configure(config_dir / "Code" / "User" / "settings.json")
