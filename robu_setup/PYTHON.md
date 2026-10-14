# Python für ROS und PlatformIO

Das Setup trennt die Werkzeuge auf **PC und Raspberry Pi**:

| Werkzeug | Python-Umgebung |
| --- | --- |
| ROS und `/usr/bin/colcon` | System-Python `/usr/bin/python3`; ROS-Pakete über APT |
| `pio` und `platformio` | Eigene Umgebung `~/.platformio/penv` |
| Ruff und weitere Entwicklungswerkzeuge | `~/work/.venvs/robu` mit `--system-site-packages` |

**Nie `~/.platformio/penv/bin` zum globalen `PATH` hinzufügen.** Sonst können
`python3` und `colcon` aus dieser Umgebung kommen; ROS fehlen dann beispielsweise
NumPy und andere Systempakete. Eine venv muss dafür nicht aktiviert sein.

Das Setup verlinkt nur die PlatformIO-Starter nach `~/.local/bin`. Diese verwenden
weiterhin ihr eigenes Python. `config/shell/tool_paths.sh` bereinigt den `PATH`
und wird aus `.bashrc` geladen. Bekannte alte Einträge werden entfernt, sonstige
Shell-Einstellungen und eigene Starter-Dateien bleiben erhalten. Die ursprüngliche
`.bashrc` wird einmalig als `.bashrc.pre-robu-tool-paths.bak` gesichert.

`microros_setup.sh` installiert fehlenden PlatformIO-Core auf PC und Pi. Alle
ROS-Builds in den angepassten Setup-Skripten verwenden ausdrücklich `/usr/bin/colcon`.
Die ROBU-Entwicklungs-venv bleibt nutzbar und kann auf die Systempakete zugreifen.
Fehler werden gemeldet; nachfolgende unabhängige Installationsschritte laufen weiter.

Bestehende Installation ohne komplettes Setup nachrüsten, **ohne sudo**:

```bash
/usr/bin/python3 ~/work/.robu/robu_setup/setup_tool_paths.py
```

Danach ein **neues Terminal** öffnen. Bereits falsch gebaute ROS-Starter behalten
ihren alten Python-Pfad und müssen neu gebaut werden. Beispiel im ROS-Workspace:

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/colcon build --packages-select turtlesim_transform --symlink-install
source install/setup.bash
```

Zur Kontrolle: `command -v colcon` sollte `/usr/bin/colcon` zeigen;
`command -v pio` zeigt `~/.local/bin/pio`. Eigene aktivierte venvs können die
Werkzeugauswahl weiterhin verändern; `/usr/bin/colcon` ist der eindeutige ROS-Aufruf.

Quellen: [PlatformIO-Starter](https://docs.platformio.org/en/latest/core/installation/shell-commands.html),
[isolierte Core-Installation](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html),
[Python-venvs und Interpreterpfade](https://docs.python.org/3.12/library/venv.html#how-venvs-work).
