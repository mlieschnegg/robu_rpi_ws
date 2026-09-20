# VS Code im Schueler-Image

`robu_setup.sh` installiert auf dem PC Python und Pylance und ruft
`setup_vscode_settings.py` als Desktop-Benutzer auf. Das gesamte Setup wie bisher
ohne `sudo` starten; einzelne Installationsschritte verwenden selbst `sudo`.

Die Vorgaben gelten fuer alle Projekte dieses Benutzers im Standardprofil:
`${XDG_CONFIG_HOME:-$HOME/.config}/Code/User/settings.json`. Sie sind keine
systemweite Sperre fuer andere Benutzer, Profile oder Remote-SSH-Installationen.
Beim Klonen des Images mit demselben Benutzer werden sie mit uebernommen.

- C/C++-Sprachdienste inklusive Tag Parser sind standardmaessig deaktiviert.
  Dadurch wird die unnoetige C++-Indizierung in Python-Projekten unterbunden.
  Der IntelliSense-Disk-Cache ist zusaetzlich ausgeschaltet.
- Python/Pylance-Indizierung und Vorschlaege bleiben aktiv. Diagnosen werden
  auf geoeffnete Dateien begrenzt. ROS-Python-Pfade fuer Ubuntu 22.04/Humble
  und 24.04/Jazzy werden ergaenzt. Bestehende zusaetzliche Pfade bleiben erhalten.
- Eigene ROS-Interfaces lassen sich weiterhin mit `colcon build` generieren.
  Nach dem Build den Workspace sourcen und VS Code daraus starten; gegebenenfalls
  die Installationspfade eigener Python-/Interface-Pakete im Projekt ergaenzen.
- Compiler, ROS-Buildwerkzeuge, PlatformIO und der C++-Debugger bleiben installiert.

Fuer C++-/PlatformIO-Unterricht kann das jeweilige Projekt in
`.vscode/settings.json` explizit einschalten:

```json
{
    "C_Cpp.intelliSenseEngine": "default",
    "C_Cpp.intelliSenseCacheSize": 256
}
```

Projekt- und Remote-Einstellungen koennen die Benutzervorgaben uebersteuern.
Das Cache-Limit begrenzt keine Symbol-Datenbanken. Auch andere VS-Code-Caches,
Erweiterungen und Build-Verzeichnisse koennen weiterhin Speicher belegen.

## Vorhandenes Image nachruesten

VS Code schliessen und als Image-Benutzer aus diesem Verzeichnis ausfuehren:

```bash
sudo apt install -y python3-json5
/usr/bin/python3 setup_vscode_settings.py
```

Vorhandene JSONC-Einstellungen (Kommentare, abschliessende Kommas) werden gelesen
und mit den Vorgaben zusammengefuehrt. Beim ersten Aendern wird die Originaldatei
einmalig als `settings.json.pre-robu.bak` gesichert. Die neue Datei wird als JSON
formatiert; Kommentare bleiben in der Sicherung. Ungueltige Einstellungen fuehren
zu einem Fehler, ohne die Originaldatei zu ueberschreiben. Wiederholtes Ausfuehren
erzeugt keine zusaetzlichen Sicherungen.

Vorhandene C++-Caches werden absichtlich nicht automatisch geloescht. Vor dem
Export des Images bei vollstaendig geschlossenem VS Code den Inhalt von
`${XDG_CACHE_HOME:-$HOME/.cache}/vscode-cpptools` pruefen und bei Bedarf entfernen.
Auch alte projektlokale `browse.vc.db`-Dateien koennen separat entfernt werden.
Keine Quelltexte oder generierten ROS-Interfaces dafuer loeschen.
