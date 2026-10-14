# VS Code im Schüler-Image

## Formatieren und Prüfen

Das Setup installiert **Ruff 0.17.0** in `~/work/.venvs/robu`. Die Version aus
`requirements-dev.txt` muss zur `required-version` im Projekt passen. Auf dem PC
wird zusätzlich **clang-format 19.1.7** installiert.

- **Strg+Umschalt+I** formatiert das Dokument.
- Python: Erweiterung `charliermarsh.ruff`; Projektregeln aus `pyproject.toml`
  oder `ruff.toml` haben Vorrang.
- C/C++: Erweiterung `ms-vscode.cpptools`; Regeln aus der nächsten `.clang-format`.
- Vier Leerzeichen; keine automatische Formatierung oder Importänderung beim Speichern.
- Formatieren ändert keine Namen. Weitere Regeln prüft `ruff check`.

Das Setup verlinkt `config/formatting` als `~/work/pyproject.toml` und
`~/work/.clang-format`. Neue Projekte darunter übernehmen die Regeln automatisch.
Vorhandene Projektkonfigurationen bleiben erhalten. Für eigene Dateien zum Committen:

```bash
/usr/bin/python3 ~/work/.robu/robu_setup/setup_formatting.py --project .
```

Details: [Formatierungsregeln](../config/formatting/README.md) und
[getrennte Python-Umgebungen für ROS und PlatformIO](PYTHON.md).

## Erweiterungen und Einstellungen

Die Erweiterungen stehen in `vscode_extensions.txt`; die IDs wurden am
14.10.2026 geprüft. ROS 2 und URDF werden einzeln installiert, das zusätzliche
RDE-Paket entfällt. Ruff und Microsoft C/C++ werden ausdrücklich installiert.

`setup_vscode_settings.py` setzt den Interpreter und Ruff aus der gemeinsamen
venv. Desktop-Einstellungen liegen unter
`${XDG_CONFIG_HOME:-$HOME/.config}/Code/User/settings.json`.
Das Setup als Benutzer **ohne sudo** starten; einzelne Installationsschritte
verwenden es selbst. Bereits ausgewählte Interpreter bei Bedarf mit
`Python: Select Interpreter` auf die ROBU-venv umstellen.

Auf dem PC bleibt Pylance aktiv. Eigene ROS-Pfade werden erhalten und Humble-/Jazzy-Pfade
ergänzt. Nach dem Build den Workspace sourcen und VS Code daraus starten.
C/C++-Indizierung ist für den Python-Unterricht deaktiviert. Im Firmware-Projekt:

```json
{
    "C_Cpp.intelliSenseEngine": "default",
    "C_Cpp.intelliSenseCacheSize": 256
}
```

## Raspberry Pi über Remote-SSH

`rpi_setup.sh` schreibt `~/.vscode-server/data/Machine/settings.json`.
Python, Ruff und C/C++ werden über `remote.SSH.defaultExtensions` auf dem SSH-Ziel
installiert. Pylance und C/C++-Indizierung bleiben auf dem Pi deaktiviert, um RAM
zu sparen. Ruff bleibt aktiv; C/C++ verwendet dort den vorhandenen bzw. mitgelieferten
Formatter. Die festgelegte clang-format-Version wird auf dem PC bereitgestellt.

Nach Änderungen Remote-SSH neu verbinden oder `Developer: Reload Window` aufrufen.
Nach dem Löschen von `.vscode-server` den Helfer erneut ausführen. Andere Serverpfade:
`VSCODE_AGENT_FOLDER` oder `--remote --server-dir /pfad/zum/server`.
Projekt- und Remote-Einstellungen können Benutzereinstellungen übersteuern.

## Bestehendes Image nachrüsten

VS Code schließen, dann als Image-Benutzer in `robu_setup`:

```bash
sudo apt install -y python3-json5 python3-venv
/usr/bin/python3 -m venv --system-site-packages "$HOME/work/.venvs/robu"
"$HOME/work/.venvs/robu/bin/python" -m pip install -r requirements-dev.txt
/usr/bin/python3 setup_formatting.py
/usr/bin/python3 setup_tool_paths.py
```

Auf dem PC zusätzlich:

```bash
"$HOME/work/.venvs/robu/bin/python" -m pip install clang-format==19.1.7
code --install-extension charliermarsh.ruff
code --install-extension ms-vscode.cpptools
/usr/bin/python3 setup_vscode_settings.py
```

Auf dem Pi: `/usr/bin/python3 setup_vscode_settings.py --remote`.
Snippets verlinken: `bash link_vsc_snippets.sh`; anschließend VS Code neu laden.
Das Snippet `ros_launch_prefix_terminal` öffnet einen ROS-Node im eigenen GNOME-Terminal.

Der Helfer erhält sonstige JSONC-Einstellungen und sichert die Originaldatei einmalig
als `settings.json.pre-robu.bak`. Ungültige Dateien werden nicht überschrieben.
Wiederholte Aufrufe sind möglich; anderer venv-Pfad: `--venv-dir /pfad/zur/venv`.

## Fehler während der Installation

Die Hauptskripte führen unabhängige Schritte nach Fehlern weiter aus und zeigen
am Ende eine Fehlerliste. Exit-Code: **0** bei Erfolg, **1** bei Fehlern.
Unterprogramme laufen in eigenen Shells. Eine fehlende venv führt nicht zu
Paketinstallationen im System-Python. Pi-Skripte verlangen weiterhin einen Pi.
Der automatische Reset von `.robu` gilt nur für den Benutzer **`robu`**;
Details in [Autostart](../autostart/README.md).

Quellen: [Ruff-Einstellungen](https://docs.astral.sh/ruff/editors/settings/),
[Ruff-Erweiterung](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff),
[ROS 2](https://marketplace.visualstudio.com/items?itemName=Ranch-Hand-Robotics.rde-ros-2),
[URDF](https://marketplace.visualstudio.com/items?itemName=Ranch-Hand-Robotics.urdf-editor),
[Microsoft C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools).
