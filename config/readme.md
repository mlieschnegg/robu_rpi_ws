# systemd-Dienste unter Ubuntu und auf dem Raspberry Pi

Ein systemd-Dienst startet ein Programm unabhängig von einer geöffneten
Terminal- oder SSH-Sitzung. Er kann beim Booten starten, seine Ausgabe ins
Journal schreiben und mit `systemctl` verwaltet werden.

Die folgenden Beispiele verwenden den Benutzer `robu`, ROS 2 Jazzy und den
Workspace `/home/robu/work/rubiks_cube`. Benutzer, Pfade und Programm müssen
zur jeweiligen Installation passen. Die Befehle werden auf dem Rechner
ausgeführt, auf dem der Dienst laufen soll, bei Bedarf über SSH.

## Fertige Vorlagen im Repository

Die Service-Vorlagen liegen im Unterordner [`service/`](service/):

| Vorlage | Aufgabe |
| --- | --- |
| [rubiks-cube.service](service/rubiks-cube.service) | ROS- und Workspace-Setup laden, dann den Rubiks-Cube-Launch starten |
| [msrobot.service](service/msrobot.service) | ROS- und `.robu`-Setup laden, dann `teleoperate_gui.py` starten |
| [robot-console.service](service/robot-console.service) | Beide Journalausgaben auf Terminal 8 anzeigen und beim Start Terminal 1 sichtbar machen |

Beide Robotervorlagen starten die gemeinsame Loganzeige bei Bedarf mit.
Die MSRobot-Vorlage wartet über `Requires` auf `/dev/ttyACM0` und
`/dev/ttyUSB0`. Benutzer, Workspace-Pfade, Plugin-Pfad und benötigte Geräte
vor dem Installieren an die jeweilige Installation anpassen.

Falls eine bestehende Installation noch den alten Namen verwendet, zuerst die
bisherige Loganzeige stoppen und ihren Autostart deaktivieren, damit nur ein
Anzeigedienst auf Terminal 8 schreibt:

```bash
sudo systemctl disable --now msrobot-console.service
```

Aus dem Wurzelverzeichnis des `.robu`-Repositories auf dem Zielrechner
installieren und prüfen:

```bash
sudo install -m 0644 \
  config/service/rubiks-cube.service \
  config/service/msrobot.service \
  config/service/robot-console.service \
  /etc/systemd/system/
sudo systemd-analyze verify \
  /etc/systemd/system/rubiks-cube.service \
  /etc/systemd/system/msrobot.service \
  /etc/systemd/system/robot-console.service
sudo systemctl daemon-reload
sudo systemctl enable --now robot-console.service
```

Den gewünschten Roboterdienst anschließend separat starten oder aktivieren,
wie unten beschrieben. Die Befehle für `rubiks-cube.service` gelten mit dem
Namen `msrobot.service` entsprechend auch für MSRobot. Wurden bereits laufende
Dienste aktualisiert, die betreffenden Dienste anschließend neu starten.

## Wo liegen die Service-Dateien?

| Verzeichnis | Verwendung |
| --- | --- |
| `/etc/systemd/system/` | Eigene Systemdienste und lokale Anpassungen |
| `/usr/lib/systemd/system/` bzw. `/lib/systemd/system/` | Dienste aus installierten Paketen |
| `/run/systemd/system/` | Temporäre Konfiguration bis zum nächsten Boot |
| `~/.config/systemd/user/` | Dienste eines einzelnen Benutzers |

Diese Anleitung beschreibt **Systemdienste** unter `/etc/systemd/system/`.
`User=robu` legt fest, unter welchem Benutzer das Programm läuft. Zum Verwalten
dieser Dienste sind meist `sudo`-Rechte nötig. Benutzerdienste werden dagegen
mit `systemctl --user` verwaltet und haben einen eigenen Lebenszyklus.

Den tatsächlichen Dateipfad und alle ergänzenden Konfigurationsdateien eines
vorhandenen Dienstes zeigt:

```bash
systemctl cat rubiks-cube.service
```

Dateien aus Paketverzeichnissen sollten über `sudo systemctl edit NAME.service`
angepasst werden, damit Paketupdates die Änderungen nicht überschreiben.

## Einen Dienst erstellen

Die Datei erhält die Endung `.service`. Beispiel:

```bash
sudo nano /etc/systemd/system/rubiks-cube.service
```

Inhalt für den Rubiks-Cube-Roboter:

```ini
[Unit]
Description=Rubiks Cube real robot (ROS 2)
Wants=network-online.target
After=network-online.target

[Service]
Type=exec
User=robu
WorkingDirectory=/home/robu/work/rubiks_cube
Environment=PYTHONUNBUFFERED=1
Environment=GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0
Environment=XDG_RUNTIME_DIR=/run/rubiks-cube
RuntimeDirectory=rubiks-cube
RuntimeDirectoryMode=0700
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source /home/robu/work/rubiks_cube/install/setup.bash && exec ros2 launch rubiks_cube_bringup real_robot.launch.py'
Restart=no
KillSignal=SIGINT
KillMode=mixed
TimeoutStopSec=30
StandardOutput=journal
StandardError=journal
SyslogIdentifier=rubiks-cube

[Install]
WantedBy=multi-user.target
```

Die wichtigsten Einstellungen:

| Einstellung | Bedeutung |
| --- | --- |
| `Description=` | Beschreibung in der Statusanzeige |
| `Wants=` und `After=` | Eine weitere Unit mitstarten und die Startreihenfolge festlegen |
| `Type=exec` | systemd überwacht den gestarteten Hauptprozess |
| `User=` | Benutzer, unter dem das Programm läuft |
| `WorkingDirectory=` | Arbeitsverzeichnis; muss vorhanden sein |
| `Environment=` | Umgebungsvariable für das Programm |
| `ExecStart=` | Befehl zum Starten des Programms |
| `Restart=no` | Nach dem Beenden nicht automatisch erneut starten |
| `KillSignal=SIGINT` | Beim Stoppen zunächst SIGINT senden, wie bei Strg+C |
| `KillMode=mixed` | Zunächst den Hauptprozess ansprechen, bei erzwungenem Abbruch alle verbliebenen Prozesse des Dienstes beenden |
| `TimeoutStopSec=30` | Frist für das reguläre Stoppen |
| `StandardOutput=` / `StandardError=` | Ziele für normale Ausgabe und Fehlermeldungen |
| `WantedBy=multi-user.target` | Ermöglicht den Autostart als Systemdienst beim Booten |

### ROS-Umgebung laden

systemd lädt keine interaktive `.bashrc`. Deshalb lädt `ExecStart` zuerst
`/opt/ros/jazzy/setup.bash` und danach `install/setup.bash` des passenden
Workspaces. Die Befehle laufen in derselben Bash; `&&` startet den nächsten
Schritt nur, wenn der vorherige erfolgreich war.

`exec` ersetzt die Bash durch `ros2 launch`, damit der Launch der überwachte
Hauptprozess ist. Ein separates `ExecStartPre=... source ...` würde die
Umgebung nicht an `ExecStart` weitergeben.

Für ROS-Programme aus dem `.robu`-Workspace lautet das Workspace-Setup
stattdessen `/home/robu/work/.robu/install/setup.bash`.

`GST_PLUGIN_PATH` entspricht dem auf dem Raspberry lokal installierten
libcamera-GStreamer-Plugin. `RuntimeDirectory` erzeugt `/run/rubiks-cube` mit
den angegebenen Zugriffsrechten für `robu`; es wird hier als `XDG_RUNTIME_DIR`
für die GUI verwendet. Diese Einstellungen sind vom gestarteten Programm
abhängig.

Für ein gewöhnliches Python-Programm kann `ExecStart` beispielsweise so aussehen:

```ini
[Service]
ExecStart=/usr/bin/python3 -u /home/robu/mein-programm/main.py
```

Diese Zeile ersetzt die bisherige `ExecStart`-Zeile im Beispiel. Beim Anpassen
über `systemctl edit` muss die ursprüngliche Zeile zuerst zurückgesetzt werden:

```ini
[Service]
ExecStart=
ExecStart=/usr/bin/python3 -u /home/robu/mein-programm/main.py
```

### Konfiguration prüfen und starten

```bash
sudo systemd-analyze verify /etc/systemd/system/rubiks-cube.service
sudo systemctl daemon-reload
sudo systemctl start rubiks-cube.service
systemctl status rubiks-cube.service --no-pager
```

Die Prüfung kontrolliert die Unit-Konfiguration. Ob ROS-Pakete, Kamera,
Motoren oder GUI funktionieren, zeigen erst der tatsächliche Start und die Logs.

## Starten, stoppen und Autostart verwalten

| Befehl | Wirkung |
| --- | --- |
| `sudo systemctl start rubiks-cube.service` | Jetzt starten |
| `sudo systemctl stop rubiks-cube.service` | Jetzt stoppen |
| `sudo systemctl restart rubiks-cube.service` | Stoppen und neu starten; startet auch einen gestoppten Dienst |
| `sudo systemctl enable rubiks-cube.service` | Autostart beim Booten aktivieren |
| `sudo systemctl disable rubiks-cube.service` | Autostart deaktivieren |
| `sudo systemctl enable --now rubiks-cube.service` | Autostart aktivieren und jetzt starten |
| `sudo systemctl disable --now rubiks-cube.service` | Autostart deaktivieren und jetzt stoppen |
| `systemctl status rubiks-cube.service` | Status und letzte Meldungen anzeigen |
| `systemctl is-active rubiks-cube.service` | Prüfen, ob der Dienst gerade aktiv ist |
| `systemctl is-enabled rubiks-cube.service` | Prüfen, ob der Autostart aktiviert ist |

**Starten und Aktivieren sind unabhängig:** `start` ändert den Autostart nicht.
`enable` startet das Programm erst mit `--now` sofort. Entsprechend stoppt
`disable` einen laufenden Dienst erst mit `--now`. Ein deaktivierter Dienst
kann weiterhin manuell oder durch eine andere Unit gestartet werden.

Zum automatischen Neustart bei Fehlern können im Abschnitt `[Service]`
statt `Restart=no` diese Einstellungen verwendet werden:

```ini
[Service]
Restart=on-failure
RestartSec=5
```

Bei Tests ohne angeschlossene Hardware ist `Restart=no` sinnvoll, damit die
gleichen Fehler nicht bei jedem erneuten Start wiederholt werden. Ein mit
`systemctl stop` gestoppter Dienst wird auch mit `Restart=on-failure` nicht
automatisch wieder gestartet.

## Ausgabe ins Journal schreiben und ansehen

Für normale Ausgabe und Fehlermeldungen im Journal:

```ini
[Service]
StandardOutput=journal
StandardError=journal
```

Die letzten Meldungen des aktuellen Boots anzeigen:

```bash
journalctl --boot --unit=rubiks-cube.service --lines=80 --no-pager
```

Live mitlesen:

```bash
journalctl --boot --follow --unit=rubiks-cube.service
```

Mehrere Dienste gemeinsam mitlesen:

```bash
journalctl --boot --follow --unit=msrobot.service --unit=rubiks-cube.service
```

Für weitere Dienste jeweils `--unit=NAME.service` ergänzen. Strg+C beendet nur
die Loganzeige, der Dienst läuft weiter. Wenn die Berechtigung zum Lesen fehlt,
kann `journalctl` mit `sudo` ausgeführt werden.

## Gemeinsame Ausgabe auf Terminal 8

Die Programme schreiben weiterhin ins Journal. Ein eigener Anzeigedienst liest
die Meldungen beider Programme und schreibt sie gemeinsam auf `/dev/tty8`.
Dadurch bleiben die Logs auch über SSH mit `journalctl` abrufbar.

Die gemeinsame Loganzeige wird als `robot-console.service` eingerichtet:

```bash
sudo nano /etc/systemd/system/robot-console.service
```

Der Anzeigedienst sammelt die Ausgaben beider Programme:

```ini
[Unit]
Description=Robot live logs on virtual console 8
After=systemd-vconsole-setup.service

[Service]
Type=simple
ExecStartPre=-/usr/bin/chvt 1
ExecStart=/usr/bin/journalctl --boot --follow --lines=40 --no-pager --unit=msrobot.service --unit=rubiks-cube.service
StandardInput=null
StandardOutput=tty
StandardError=journal
TTYPath=/dev/tty8
TTYReset=yes
TTYVTDisallocate=yes
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

`TTYPath=/dev/tty8` bestimmt das Ausgabeziel, weil `StandardOutput=tty` gesetzt
ist. `TTYReset` setzt die Terminal-Einstellungen zurück, `TTYVTDisallocate`
leert beim Start und Stoppen des Anzeigedienstes den Bildschirm und Scrollback.
Der Anzeigedienst läuft als root, um die Konsole öffnen und umschalten zu können;
die Roboterprogramme selbst laufen weiterhin als `robu`.

Nach dem erstmaligen Erstellen prüfen und aktivieren:

```bash
sudo systemd-analyze verify /etc/systemd/system/robot-console.service
sudo systemctl daemon-reload
sudo systemctl enable --now robot-console.service
```

Bei einer Änderung an einem bereits laufenden Anzeigedienst anschließend
`sudo systemctl restart robot-console.service` ausführen.

Damit die Loganzeige auch bei einem manuellen Start des Roboters mitgestartet
wird, kann im vorhandenen Abschnitt `[Unit]` von `rubiks-cube.service` die
`Wants`-Zeile erweitert werden:

```ini
[Unit]
Wants=network-online.target robot-console.service
```

### Login auf Terminal 1, Logs auf Terminal 8

| Terminal | Verwendung |
| --- | --- |
| `/dev/tty1` | Normaler lokaler Login |
| `/dev/tty2` bis `/dev/tty6` | Weitere Login-Konsolen bei Bedarf |
| `/dev/tty8` | Gemeinsame Roboterausgaben |

Systemd sieht standardmäßig die ersten sechs virtuellen Konsolen für
automatische Login-Prompts vor. Auf dem Raspberry ist Terminal 8 deshalb für
die Logs vorgesehen. Es gibt keine besondere Regel für Ausgaben ab Terminal 10.
Bei einer anderen Konsole muss geprüft werden, ob dort bereits ein Login-Dienst
(`getty@ttyN.service`) oder eine grafische Sitzung läuft.

`ExecStartPre=-/usr/bin/chvt 1` schaltet beim Start des Anzeigedienstes auf
Terminal 1. So erscheint beim Booten der Login, während die Logs im Hintergrund
auf Terminal 8 geschrieben werden. Das vorangestellte `-` erlaubt, dass der
Anzeigedienst auch bei fehlgeschlagener Umschaltung startet.

Mit einer angeschlossenen Tastatur wechselt man normalerweise mit **Alt+F1**
zum Login und mit **Alt+F8** zu den Logs. Aus einer grafischen Sitzung kann
**Strg+Alt+F1** bzw. **Strg+Alt+F8** nötig sein. Alternativ:

```bash
sudo chvt 1
sudo chvt 8
```

`chvt` schaltet den lokal angeschlossenen Bildschirm um, auch wenn der Befehl
über SSH ausgeführt wird. Die eigene SSH-Sitzung bleibt davon unabhängig.

Zum Ändern des Ausgabeziels beispielsweise auf Terminal 9 nur
`TTYPath=/dev/tty9` anpassen. `chvt 1` bleibt bestehen, solange beim Start der
Login sichtbar sein soll. Soll stattdessen die Logkonsole sofort sichtbar
werden, zusätzlich `chvt 9` einstellen.

## Andere Ausgabeziele

### Ein Programm direkt auf eine Konsole schreiben lassen

Im Abschnitt `[Service]` des betreffenden Programms:

```ini
[Service]
StandardInput=null
StandardOutput=tty
StandardError=tty
TTYPath=/dev/tty8
```

Bei dieser Variante landen die Ausgaben direkt auf dem Terminal und werden
nicht automatisch zusätzlich ins Journal geschrieben. Schreiben mehrere
Programme gleichzeitig auf dasselbe Terminal, können sich ihre Ausgaben
vermischen. Für gemeinsame Logs ist daher der oben gezeigte Anzeigedienst
sinnvoll.

### Ausgabe an eine Datei anhängen

```ini
[Service]
StandardOutput=append:/var/log/mein-programm.log
StandardError=inherit
```

`append:` hängt neue Ausgaben an die Datei an. `StandardError=inherit` verwendet
dasselbe Ziel wie die normale Ausgabe. Auch hier wird diese Ausgabe nicht
automatisch zusätzlich ins Journal geschrieben.

## Änderungen übernehmen und Fehler finden

Nach Änderungen an einer Service-Datei:

```bash
sudo systemd-analyze verify /etc/systemd/system/rubiks-cube.service
sudo systemctl daemon-reload
sudo systemctl restart rubiks-cube.service
```

Für Änderungen am Anzeigedienst entsprechend `robot-console.service`
verwenden. `daemon-reload` liest die Unit-Dateien neu ein; ein bereits laufendes
Programm übernimmt geänderte Startparameter erst beim nächsten Start.
`systemctl reload` lädt dagegen die eigene Konfiguration eines Programms und
wird nur von dafür eingerichteten Diensten unterstützt.

Zur Fehlersuche:

```bash
systemctl status rubiks-cube.service --no-pager
systemctl cat rubiks-cube.service
journalctl --boot --unit=rubiks-cube.service --lines=100 --no-pager
```

| Meldung oder Zustand | Was prüfen? |
| --- | --- |
| `status=200/CHDIR` | Existiert `WorkingDirectory` und darf der Benutzer darauf zugreifen? |
| `ros2: command not found` | Wird `/opt/ros/jazzy/setup.bash` geladen? |
| `Package ... not found` | Wird das Setup des richtigen, bereits gebauten Workspaces geladen? |
| `Start request repeated too quickly` | Fehler in den Logs beheben, dann das Startlimit zurücksetzen |
| `inactive (dead)` | Hat sich das Programm beendet? Exitcode und Logs prüfen |

Nach dem Beheben eines Fehlers bei erreichtem Startlimit:

```bash
sudo systemctl reset-failed rubiks-cube.service
sudo systemctl start rubiks-cube.service
```

Beim Test des Rubiks-Cube-Launches ohne Monitor, Kamera und Motoren sind
entsprechende Gerätefehler zu erwarten. Der Launch kann seine Nodes daraufhin
beenden und trotzdem mit Exitcode 0 zurückkehren. Ein erfolgreicher
`systemctl start` oder `Result=success` bestätigt daher nicht die Funktion der
Hardware. Der separate Anzeigedienst bleibt aktiv und zeigt die Fehlermeldungen
weiterhin auf Terminal 8 an.

## Weiterführende Dokumentation

- [systemctl: Dienste verwalten](https://www.freedesktop.org/software/systemd/man/latest/systemctl.html)
- [systemd.service: Aufbau eines Dienstes](https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html)
- [systemd.exec: Umgebung und Ausgabeziele](https://www.freedesktop.org/software/systemd/man/latest/systemd.exec.html)
- [journalctl: Logs anzeigen und filtern](https://www.freedesktop.org/software/systemd/man/latest/journalctl.html)
- [logind.conf: Virtuelle Login-Konsolen](https://www.freedesktop.org/software/systemd/man/latest/logind.conf.html)
