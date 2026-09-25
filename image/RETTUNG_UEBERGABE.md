# Übergabe: vorhandene Raspberry-Pi-Installation retten

Ziel ist, die bestehende Installation mit Programmen und Konfiguration zu erhalten
und ein überprüftes Image für eine neue Karte zu erzeugen. Das Skript prüft und
verkleinert; es ist **kein automatisches Reparaturwerkzeug**.

## Ausgangslage

Ein ddrescue-Rettungsimage wurde bereits erstellt. Frühere Klonversuche brachen
beim Vergleich ab. Auch wiederholte Lesungen des ersten GiB lieferten teilweise
unterschiedliche Prüfsummen. Die Ursache ist noch ungeklärt; ein Kartendefekt ist
nicht bewiesen. Dass die Originalkarte bootet, bestätigt nicht alle Dateiinhalte.

Originalkarte, Rettungsimage und zugehörige ddrescue-Map aufbewahren. Die Karte
bis zum Ende der Diagnose nicht booten oder beschreiben. Das Rettungsimage nicht
mit PiShrink oder reparierendem fsck direkt bearbeiten. Das Vorbereitungsskript
`prepare_pi_for_backup.sh` gehört nicht zu diesem Rettungsablauf: Es löscht Daten
und aktualisiert Pakete auf dem System, auf dem es gestartet wird.

## 1. Geräte und Dateien bestimmen

Im Repository-Verzeichnis `image` arbeiten. Hilfe:

```bash
./make_pi_image.sh --help
lsblk -o NAME,SIZE,MODEL,FSTYPE,MOUNTPOINTS
```

Nach jedem Umstecken die Gerätenamen erneut kontrollieren. `/dev/mmcblk0` ist
häufig der interne Leser, `/dev/sdb` kann ein externer Leser sein; beides sind
keine garantierten Zuordnungen. Quelle, SSD und neue Zielkarte unterscheiden!
Alle Partitionen der Originalkarte einzeln aushängen. Desktop-Automount darf sie
während der Prüfung nicht erneut einhängen. Die SSD mit dem Image bleibt gemountet.

Die folgenden Beispiele verwenden den bisherigen Pfad und internen Leser.
**Vor Ausführung an tatsächlichen Image-Pfad und Originalkarte anpassen.**
Der ddrescue-Prozess muss beendet sein; niemand darf parallel das Image ändern.
Die `.map`-Datei dokumentiert die Rettung; das Image-Skript wertet sie nicht aus.

## 2. Vorhandenes Rettungsimage prüfen

```bash
sudo ./make_pi_image.sh --check-existing \
  -d /dev/mmcblk0 \
  -o /media/mlieschnegg/LI/rpi_image/robot-server-rescue.img
```

Jeder Abschnitt von 1 GiB wird mit einer neuen direkten Lesung der Originalkarte
verglichen. Es wird nichts in das Rettungsimage geschrieben. Der letzte Abschnitt
kann kleiner sein. Danach folgen Prüfungen der Partitionstabelle und von FAT/ext4.
Nur bei vollständigem Erfolg entstehen `.sha256` und `.bytes` neben dem Image.
Diese gemeinsam mit dem Image behalten; nicht selbst erzeugen, um eine
fehlgeschlagene Prüfung zu umgehen.

Ein einzelner erfolgreicher Abschnitt reicht nicht. Es gibt keine automatischen
Wiederholungen und keine Wiederaufnahme einer teilweise bestandenen Prüfung.
Ein neuer Aufruf startet von vorn. Nach bereits erfolgreicher Übernahme nicht
nochmals `--check-existing` aufrufen: Die Begleitdateien verhindern Überschreiben.

## 3. Bei einem Fehler

| Meldung | Bedeutung und nächster Schritt |
| --- | --- |
| Abschnitt nicht identisch / Lesefehler | Stoppen. Komplette Ausgabe, Abschnitt und Byte-Offset sichern. Image und Original unverändert lassen. Gerätezuordnung, Mountzustand und Kernelmeldungen prüfen. Nicht automatisch die zuletzt gelesene Variante übernehmen. |
| Image und Quelle nicht gleich groß | Gerätezuordnung und Vollständigkeit der Rettung prüfen. Nicht durch Abschneiden/Vergrößern passend machen. |
| FAT/ext4-Prüfung fehlgeschlagen | Quelle stimmt möglicherweise mit dem Image überein, aber Dateisystemprüfung besteht nicht. Vor Reparatur eine separate Arbeitskopie anlegen; keine Reparatur der Originalkarte oder des einzigen Rettungsimages. |
| Zu wenig Speicher | Platz auf der SSD schaffen. Das vorhandene Rettungsimage bleibt erhalten. |
| Datei existiert bereits | Vorhandene Ergebnisse identifizieren. Nicht blind löschen. Bei geprüfter Rohkopie ist für Shrink `--resume` vorgesehen. |
| Shrink fehlgeschlagen | Die geprüfte Rohkopie bleibt erhalten. Ursache anhand der Ausgabe klären, dann `--resume`. |

Für die Übergabe eines Problems festhalten: verwendeter Befehl, Gerätenamen und
Modelle, `lsblk`-Ausgabe, vollständige Fehlermeldung, Zeitpunkt und Kernelmeldungen:

```bash
sudo journalctl -k -b --no-pager
```

Nicht nur die letzte Fehlerzeile aufheben. Die Abschnittsangabe nennt den absoluten
Startoffset ab Byte 0; eine `cmp`-Angabe innerhalb dieses Abschnitts ist relativ.

Bei wechselnden Lesedaten muss vor weiteren Änderungen die Betreuungslehrkraft
hinzugezogen werden. Zwei gleiche Lesungen beweisen nicht, dass die Daten richtig
sind. Reparaturen können Dateien verlieren oder verändern. Ein erfolgreicher fsck
bestätigt Dateisystemstrukturen, nicht den Inhalt jeder Datei.

**Grenze des Skripts:** Eine reparierte Arbeitskopie ist absichtlich nicht mehr
bytegleich zur Originalkarte. Der jetzige Übernahmemodus ist dafür nicht geeignet.
Nicht Prüfungen umgehen oder `.sha256`/`.bytes` selbst als Freigabe erzeugen;
die reparierte Kopie benötigt einen gesonderten geprüften Übernahmeweg.

## 4. Nach vollständig erfolgreicher Prüfung verkleinern

Voraussetzung: PiShrink ist installiert und als `pishrink.sh` im Suchpfad verfügbar.
Die [offizielle PiShrink-Installationsanleitung](https://github.com/Drewsif/PiShrink#installation)
beschreibt Download und Installation. Für die reine Prüfung in Schritt 2 ist
PiShrink nicht erforderlich.

Jetzt ist die Originalkarte für diesen Schritt nicht mehr erforderlich:

```bash
sudo ./make_pi_image.sh --resume \
  -o /media/mlieschnegg/LI/rpi_image/robot-server-rescue.img
```

PiShrink verändert eine separate Arbeitskopie. Ergebnis sind
`robot-server-rescue.img.gz`, `.img.gz.sha256` und `.img.gz.bytes`.
Das ursprüngliche Rettungsimage bleibt erhalten. `.bytes` nennt die benötigte
unkomprimierte Kapazität; die kleine gzip-Dateigröße ist dafür nicht maßgeblich.

## 5. Neue Karte schreiben und vor dem ersten Boot vergleichen

Mit einem Image-Writer das `.img.gz` auf eine **neue** Karte schreiben. Dabei wird
diese Zielkarte überschrieben. Vor dem Bestätigen im Writer Gerätezuordnung und
Kapazität prüfen. Das Skript selbst schreibt keine Zielkarten.

Zielkarte sicher auswerfen, neu anstecken, Gerätenamen erneut feststellen und
alle Zielpartitionen aushängen. Beispiel für eine Zielkarte unter `/dev/sdY`
(Platzhalter ersetzen):

```bash
sudo ./make_pi_image.sh --verify /dev/sdY \
  -o /media/mlieschnegg/LI/rpi_image/robot-server-rescue.img.gz
```

Nur nach erfolgreichem Vergleich booten. Ein Boot verändert die Karte, danach
ist dieser exakte Vergleich nicht mehr sinnvoll.

## 6. Funktionstest und Abschluss

Den ersten Start und mögliche automatische Neustarts abwarten. Prüfen:

```bash
lsblk
df -h /
systemctl --failed
```

Die Root-Partition und ihr Dateisystem sollen nach Autoexpand den verfügbaren
Kartenplatz nutzen. Autoexpand hängt vom gestarteten System ab und ist durch die
Image-Prüfung nicht getestet. Die benötigten Robotik-Dienste und Programme
praktisch starten und deren Konfiguration kontrollieren.

Ergebnisse dokumentieren: Vergleich erfolgreich, Erweiterung erfolgreich,
getestete Funktionen und verbleibende Fehler. Originalkarte, Rohimage und Map
bis zur Abnahme aufbewahren. Vor Weitergabe auch persönliche Zugänge prüfen;
GitHub-Konto `techtitans-htlk` darf bleiben, `mlieschnegg` nicht.
