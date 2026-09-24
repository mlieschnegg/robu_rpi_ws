# SD-Karte sichern und Rückschreiben prüfen

Die Quelle muss sauber heruntergefahren sein. Die SD-Karte an einem anderen
Linux-Rechner lesen und **alle** Partitionen aushängen; Desktop-Automount während
der Sicherung deaktivieren. Kein Live-Backup des laufenden Raspberry Pi.
`sync` allein ersetzt weder Herunterfahren noch Aushängen.

Benötigt: Bash, Python 3, util-linux, coreutils, e2fsprogs, dosfstools, gzip;
für `--shrink` zusätzlich ein lokal installiertes, vertrauenswürdiges PiShrink.
Unterstützt wird das übliche Layout mit DOS/MBR, FAT als erster und ext4 als
zweiter Partition, mit 512-Byte-Sektoren. Andere Layouts werden abgelehnt.

```bash
sudo ./make_pi_image.sh -d /dev/sdX -o /mnt/ssd/raspi.img --shrink
```

Die Sicherung überschreibt keine vorhandenen Ausgaben. Sie liest die Quelle
zweimal, vergleicht die Rohkopie und prüft Partitionierungsgrenzen sowie FAT/ext4.
Danach bleibt die geprüfte Rohkopie als `raspi.img` mit `.sha256` und `.bytes`
dauerhaft erhalten. PiShrink verändert ausschließlich eine separate Arbeitskopie.
Diese wird erneut geprüft und erst danach komprimiert. Die erfolgreiche Ausgabe
heißt `raspi.img.gz`; die Rohkopie wird auch bei Erfolg nicht automatisch gelöscht.

### Shrink nach einem Fehler wiederholen

Wenn PiShrink abbricht, das verkleinerte Dateisystem beschädigt ist oder die
Komprimierung scheitert, bleiben Rohkopie und deren Begleitdateien erhalten.
Fehlerhafte temporäre Arbeitskopien werden entfernt. Nach Behebung der Ursache:

```bash
sudo ./make_pi_image.sh --resume -o /mnt/ssd/raspi.img
```

Dabei wird **kein dd ausgeführt und die SD-Karte nicht benötigt**. Das Skript
prüft Größe, SHA256 und Dateisysteme der gespeicherten Rohkopie und versucht
Shrink und Komprimierung auf einer neuen Arbeitskopie. Auch eine ohne `--shrink`
erstellte geprüfte Rohkopie lässt sich so nachträglich verkleinern.
Ein Fehler beim ursprünglichen Lesen, Quellvergleich oder der ersten
Dateisystemprüfung erzeugt dagegen keine freigegebene Rohkopie für `--resume`.

Ohne Shrink wird vorsorglich Platz für die ganze Karte plus 1 GiB verlangt.
Mit Shrink sind es drei Kartengrößen plus 1 GiB (Rohkopie, Arbeitskopie und
komprimierte Ausgabe); beim Wiederaufnehmen zwei zusätzliche Kartengrößen plus
1 GiB. Reflinks werden genutzt, wenn das SSD-Dateisystem sie unterstützt,
sodass die Arbeitskopie dort zunächst wenig zusätzlichen Platz benötigt.
Die konservative Platzprüfung setzt diese Unterstützung nicht voraus.

Fehler werden nicht automatisch repariert. Eine vorhandene endgültige
`.gz`-Ausgabe oder deren Begleitdateien werden auch mit `--resume` nicht
überschrieben. Falls ein harter Abbruch während der Veröffentlichung einzelne
Ergebnisdateien hinterlassen hat, diese vor einem neuen Versuch separat
verschieben; `raspi.img`, `raspi.img.sha256` und `raspi.img.bytes` behalten.

`.sha256` schützt die Image-Datei bei späterer Lagerung/Übertragung; `.bytes`
enthält die erforderliche **unkomprimierte** Mindestkapazität. Die gzip-Größe
sagt nicht, ob das Image auf eine 32-GB-Karte passt. PiShrink richtet standardmäßig
die Erweiterung beim ersten Boot ein; deren Erfolg anschließend am Pi mit
`df -h /` und `lsblk` prüfen.

Nach dem Schreiben mit einem Image-Writer die Karte sicher auswerfen und neu
anstecken, alle Partitionen wieder aushängen und **vor dem ersten Boot** prüfen:

```bash
sudo ./make_pi_image.sh --verify /dev/sdY -o /mnt/ssd/raspi.img.gz
```

Dieser Modus schreibt kein Image, sondern prüft Prüfsumme, Kartenkapazität und
vergleicht den gesamten Image-Bereich byteweise. Zusätzlicher Platz auf der
Zielkarte ist erlaubt. Image und beide Begleitdateien zusammen aufbewahren.
Ein Boot verändert die Karte und macht den exakten Vergleich unbrauchbar.

Dateisystemprüfungen prüfen Metadaten, nicht die inhaltliche Richtigkeit jeder
Datei. Bereits beschädigte Dateien können auch in einem strukturell sauberen
Dateisystem liegen. Wiederkehrende Abstürze trotz erfolgreicher Prüfung können
auch Karte, Kartenleser, Stromversorgung oder Hardware betreffen. Kernelmeldungen
am Pi mit `journalctl -k -b` auf I/O-, MMC- und ext4-Fehler prüfen. Bei wichtigen
Daten vor Reparaturen eine unveränderte Sicherung aufbewahren.

## Vorbereitung und Bash-History

`prepare_pi_for_backup.sh` wird als normaler Benutzer ohne vorangestelltes
`sudo` gestartet. Persönliche Dateien werden mit Benutzerrechten im eigenen
Home-Verzeichnis bereinigt. Paketpflege, Snap-Verwaltung, Systemprüfung und
Herunterfahren verwenden intern `sudo`; zu Beginn wird die Berechtigung geprüft
und gegebenenfalls nach dem Passwort gefragt. Ein Start als root wird abgelehnt.

Das Skript löscht die gespeicherte Standard-Bash-History dieses Benutzers. Ein Shell-Skript kann die
History-Liste seiner Eltern-Shell nicht leeren. Jede noch laufende Bash kann
beim Beenden oder über `PROMPT_COMMAND` alte Einträge wieder schreiben.

Andere Sitzungen zuerst schließen. Dann in der letzten **interaktiven Bash**
(dies betrifft auch root-Sitzungen, falls deren History entfernt werden soll):

```bash
history -c
history -w
unset HISTFILE
./prepare_pi_for_backup.sh
```

Bei einer eigenen History-Konfiguration deren Speicherpfad/Prompt-Hooks ebenfalls
beachten. Diese Bereinigung ist keine sichere Löschung alter Datenblöcke.
Auch die übrigen Bereinigungen betreffen jetzt das eigene Home-Verzeichnis,
insbesondere Downloads, Firefox-Snap-Daten, Caches und die aufgeführten SSH-Dateien.
Das Skript führt außerdem Paket-Upgrades aus. Vor Benutzung lesen.
`--no-shutdown` lässt das System laufen: Die SD-Karte darf dann nicht entfernt werden.

## GitHub-Zugang vor Weitergabe

Das Vorbereitungsskript benötigt `gh` mit Unterstützung für
`gh auth status --json hosts`, Python 3 und Git. Es prüft alle in der aktuellen
GitHub-CLI-Konfiguration gespeicherten github.com-Konten (auch inaktive), meldet
`mlieschnegg` mit `gh auth logout` lokal ab und kontrolliert den Status erneut.
Konfigurierte Git-Credential-Helper werden über `git credential reject` zum
Entfernen des HTTPS-Zugangs für `mlieschnegg@github.com` aufgefordert.
Bei fehlenden Werkzeugen, unlesbarem Status, fehlgeschlagener Abmeldung oder
verbleibendem Konto stoppt das Skript vor den weiteren Bereinigungen.

Gesetzte `GH_TOKEN`/`GITHUB_TOKEN` werden für die CLI-Prüfung ausgeblendet, damit
sie gespeicherte Konten nicht verdecken. Sind solche Variablen vorhanden, stoppt
das Skript nach der lokalen Abmeldung: Sie müssen auch aus der aufrufenden Shell
und ihrer dauerhaften Konfiguration entfernt werden. Tokenwerte werden nicht ausgegeben.

Das ist keine vollständige Bereinigung aller GitHub-Zugangswege: Browser- und
VS-Code-Sitzungen, private SSH-Schlüssel, andere Benutzer (einschließlich root),
andere CLI-Konfigurationsverzeichnisse und Tokens in Dateien/Repository-URLs
sind nicht durch `gh auth logout` abgedeckt. Auch Credential-Helper können unter
anderen Benutzernamen oder Repository-Pfaden gespeicherte Einträge behalten.
Diese Zugänge vor Weitergabe separat entfernen. Die bisherige Löschung von
`authorized_keys` und `known_hosts` entfernt **keine privaten SSH-Schlüssel**.
Die lokale Abmeldung widerruft den Token nicht serverseitig; falls bereits eine
Kopie mit Zugangsdaten weitergegeben wurde, den Zugang bei GitHub widerrufen.

Referenzen: [gh auth status](https://cli.github.com/manual/gh_auth_status),
[gh auth logout](https://cli.github.com/manual/gh_auth_logout),
[git credential](https://git-scm.com/docs/git-credential).
