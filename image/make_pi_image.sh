#!/usr/bin/env bash
# Offline-Backup einer SD-Karte. Niemals auf dem laufenden Quellsystem ausführen.
set -Eeuo pipefail
export LC_ALL=C
usage() {
  cat <<'USAGE'
SD-Image erstellen, ein Rettungsimage prüfen oder eine Zielkarte vergleichen

Aufrufe (im Verzeichnis image):
  sudo ./make_pi_image.sh -d /dev/sdX -o backup.img [--shrink]
  sudo ./make_pi_image.sh --check-existing -d /dev/sdX -o rescue.img [--shrink]
  sudo ./make_pi_image.sh --resume -o backup.img
  sudo ./make_pi_image.sh --verify /dev/sdY -o backup.img[.gz]

Optionen:
  -d GERÄT         Ganze Originalkarte, nicht eine Partition. Alle Partitionen
                   müssen ausgehängt sein und während des Laufs so bleiben.
  -o DATEI         Neue .img-Datei bei einer Sicherung; vorhandene Datei bei
                   --check-existing, --resume und --verify.
  --check-existing Vorhandenes vollständiges Roh-/ddrescue-Image mit der
                   Originalkarte vergleichen. Keine erneute Vollkopie.
  --shrink         Nach erfolgreicher Prüfung eine Arbeitskopie mit PiShrink
                   verkleinern, erneut prüfen und als .img.gz komprimieren.
                   Das ursprüngliche Rohimage bleibt erhalten.
  --resume         Geprüftes Rohimage mit .sha256 und .bytes wiederverwenden
                   und Shrink versuchen. Originalkarte nicht erforderlich.
                   Setzt KEINE abgebrochene Kopie/Abschnittsprüfung fort.
  --verify GERÄT   Bereits beschriebene NEUE Karte vor dem ersten Boot mit dem
                   fertigen Image vergleichen. Schreibt nichts auf die Karte.
  -h, --help       Diese Hilfe anzeigen (ohne sudo möglich).

Vergleich nach jedem GiB:
  Neue Sicherung: jeweils 1 GiB kopieren, synchronisieren und sofort mit einer
  erneuten direkten Lesung der Originalkarte vergleichen. Letzter Teil kürzer.
  --check-existing: vorhandenes Image in denselben Abschnitten vergleichen.
  1 GiB = 1.073.741.824 Bytes. Bei Abweichung/Lesefehler sofortiger Abbruch mit
  Abschnitt und Byte-Offset. Keine automatischen Wiederholungen oder Reparaturen.
  Der gemeldete Abschnitts-Offset beginnt bei 0; cmp meldet die abweichende Stelle
  relativ zum verglichenen Abschnitt. Das Rettungsimage wird nicht verändert.

Ablauf für ein bereits erstelltes ddrescue-Image:
  1. ddrescue beenden. Originalkarte und Rettungsimage unverändert lassen.
     Image und zugehörige .map-Datei behalten. Gerätenamen mit lsblk prüfen.
  2. sudo ./make_pi_image.sh --check-existing -d /dev/sdX -o rescue.img
     Erst wenn ALLE Abschnitte und FAT/ext4-Prüfungen erfolgreich sind, entstehen
     rescue.img.sha256 und rescue.img.bytes. Die ddrescue-Map wird nicht geprüft.
  3. sudo ./make_pi_image.sh --resume -o rescue.img
     Ergebnis: rescue.img.gz mit eigenen .sha256- und .bytes-Dateien.
  4. .img.gz mit einem Image-Writer auf eine NEUE ausreichend große Karte schreiben.
     Dabei wird die neue Karte überschrieben. Dieses Skript übernimmt das nicht.
  5. Neue Karte sicher auswerfen, neu anstecken, Partitionen aushängen; vor Boot:
     sudo ./make_pi_image.sh --verify /dev/sdY -o rescue.img.gz
  6. Erst bei Erfolg im Pi booten und Funktionen sowie Erweiterung prüfen (df -h /).

Bei Fehlern:
  Vergleich/Dateisystemprüfung fehlgeschlagen: Ausgabe sichern, nicht shrinken,
  Original und Rettungsimage nicht reparieren. Diagnose/Reparatur nur auf einer
  separaten Arbeitskopie. Begleitdateien niemals selbst als Freigabe erzeugen.
  Neue temporäre Rohkopien werden bei Abbruch gelöscht; vorhandene Rettungsimages
  bleiben erhalten. Eine Abschnittsprüfung beginnt beim nächsten Aufruf von vorn.
  Shrink/Platzproblem: geprüfte Rohkopie bleibt erhalten; Ursache beheben, --resume.
  Vorhandene Ausgaben werden nicht überschrieben. Eine Reparatur kann Image-Bytes
  verändern: danach kein Originalkartenvergleich als Erfolgskriterium.

Unterstützt: DOS/MBR, FAT-Bootpartition + ext4-Rootpartition, 512-Byte-Sektoren.
Platzprüfung pro Arbeitsschritt plus 1 GiB Reserve; gzip mit kleinem Größenzuschlag.
Ein erfolgreicher Vergleich beweist keine inhaltliche Fehlerfreiheit aller Dateien.
PiShrink installieren (offizielle Anleitung):
  https://github.com/Drewsif/PiShrink#installation
Schüler-Übergabe mit Fehlerbehandlung: RETTUNG_UEBERGABE.md neben diesem Skript.

USAGE
}
die() { echo "FEHLER: $*" >&2; exit 1; }
SRC='' OUT='' VERIFY='' LOOP='' WORK='' PACKED='' RAW_READY=false SHRINK=false RESUME=false EXISTING=false
while (($#)); do
  case "$1" in
    -d|-o|--verify)
      (($# >= 2)) && [[ -n "$2" ]] || die "Argument für $1 fehlt."
      case "$1" in -d) SRC=$2;; -o) OUT=$2;; --verify) VERIFY=$2;; esac
      shift 2;;
    --shrink) SHRINK=true; shift;;
    --check-existing) EXISTING=true; shift;;
    --resume) RESUME=true; SHRINK=true; shift;;
    -h|--help) usage; exit 0;;
    *) die "Unbekannte Option: $1";;
  esac
done
[[ -n "$OUT" ]] || { usage; exit 1; }
[[ $EUID == 0 ]] || die 'Bitte mit sudo ausführen.'
for tool in lsblk findmnt swapon blockdev losetup sfdisk python3 blkid e2fsck fsck.fat cmp dd gzip sha256sum stat realpath mktemp df cp mv sync; do
  command -v "$tool" >/dev/null || die "Benötigtes Werkzeug fehlt: $tool"
done
cleanup() {
  local status=$?
  if [[ -n "$LOOP" ]]; then losetup -d "$LOOP" || true; fi
  if [[ -n "$WORK" ]]; then rm -f -- "$WORK"; fi
  if [[ -n "$PACKED" ]]; then rm -f -- "$PACKED" "$PACKED.bytes" "$PACKED.sha256"; fi
  if ((status != 0)) && $RAW_READY; then
    echo "Die geprüfte Rohkopie bleibt erhalten: $OUT" >&2
    printf 'Shrink erneut starten: sudo %q --resume -o %q\n' "$0" "$OUT" >&2
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'echo "FEHLER: Abbruch in Zeile $LINENO. Der angeforderte Ablauf wurde nicht abgeschlossen." >&2' ERR
check_device() {
  local dev=$1 node type
  [[ -b "$dev" ]] || die "Kein Blockgerät: $dev"
  [[ $(lsblk -dnro TYPE "$dev") == disk ]] || die 'Bitte ein ganzes physisches Laufwerk angeben.'
  while read -r node type; do
    [[ "$type" == disk || "$type" == part ]] || die "Aktives Mapping auf $node ($type)."
    if findmnt -rn -S "$node" >/dev/null; then die "$node ist eingehängt (auch read-only ist nicht erlaubt)."; fi
    while read -r swap; do
      [[ $(realpath "$swap") != "$node" ]] || die "$node wird als Swap verwendet."
    done < <(swapon --noheadings --raw --show=NAME)
  done < <(lsblk -nrpo NAME,TYPE "$dev")
  [[ $(blockdev --getss "$dev") == 512 ]] || die 'Nur Geräte mit 512-Byte-Sektoren unterstützt.'
}
check_image() {
  local file=$1 part fs
  echo '==> Prüfe Partitionstabelle und Grenzen ...'
  sfdisk --json "$file" | python3 -c '
import json, os, sys
p=json.load(sys.stdin)["partitiontable"]
a=p["partitions"]; sector=p.get("sectorsize",512)
if p["label"] != "dos" or sector != 512 or len(a) != 2:
    sys.exit("Nur DOS/MBR mit zwei Partitionen unterstützt")
end=1
for x in a:
    start=x["start"]; size=x["size"]
    if size <= 0 or start < end or (start+size)*sector > os.path.getsize(sys.argv[1]):
        sys.exit("Ungültige/überlappende Partition oder Image abgeschnitten")
    end=start+size
' "$file"
  LOOP=$(losetup --find --show --read-only --partscan "$file")
  for part in 1 2; do
    [[ -b "${LOOP}p${part}" ]] || die "Partition $part fehlt."
    fs=$(blkid -p -s TYPE -o value "${LOOP}p${part}")
    if [[ "$part" == 1 && "$fs" == vfat ]]; then
      fsck.fat -n "${LOOP}p${part}" || die 'Boot-Dateisystem fehlerhaft. Prüfung fehlgeschlagen; dieses Image nicht verwenden.'
    elif [[ "$part" == 2 && "$fs" == ext4 ]]; then
      e2fsck -f -n "${LOOP}p${part}" || die 'Root-Dateisystem fehlerhaft. Prüfung fehlgeschlagen; dieses Image nicht verwenden.'
    else
      die "Nicht unterstütztes Dateisystem auf Partition $part: $fs"
    fi
  done
  losetup -d "$LOOP"; LOOP=''
}
OUT=$(realpath -m "$OUT")
if [[ -n "$VERIFY" ]]; then
  [[ -z "$SRC" && "$SHRINK" == false && "$RESUME" == false && "$EXISTING" == false ]] || die '--verify nicht mit -d/--shrink kombinieren.'
  VERIFY=$(realpath "$VERIFY")
  check_device "$VERIFY"
  [[ -f "$OUT" && -f "$OUT.sha256" && -f "$OUT.bytes" ]] || die 'Image, .sha256 oder .bytes fehlt.'
  (cd "$(dirname "$OUT")"; sha256sum -c -- "$(basename "$OUT").sha256")
  bytes=$(cat "$OUT.bytes")
  [[ "$bytes" =~ ^[1-9][0-9]*$ ]] || die 'Ungültige Image-Größe.'
  (( $(blockdev --getsize64 "$VERIFY") >= bytes )) || die 'Zielkarte ist zu klein.'
  blockdev --flushbufs "$VERIFY"
  echo '==> Vergleiche alle Image-Bytes mit der Zielkarte ...'
  if [[ "$OUT" == *.gz ]]; then
    gzip -dc -- "$OUT" | cmp -n "$bytes" - "$VERIFY"
  else
    [[ $(stat -c %s "$OUT") == "$bytes" ]] || die 'Image-Größe stimmt nicht.'
    cmp -n "$bytes" -- "$OUT" "$VERIFY"
  fi
  echo '==> Zielkarte stimmt mit dem geprüften Image überein.'
  exit 0
fi
[[ "$OUT" == *.img ]] || die 'Ausgabename muss auf .img enden.'
if $RESUME; then
  $EXISTING && die '--resume nicht mit --check-existing kombinieren.'
  [[ -z "$SRC" ]] || die '--resume nicht mit -d kombinieren.'
  [[ -f "$OUT" && -f "$OUT.sha256" && -f "$OUT.bytes" ]] || die 'Geprüfte Rohkopie mit .sha256 und .bytes fehlt.'
else
  [[ -n "$SRC" ]] || die '-d fehlt.'
  SRC=$(realpath "$SRC")
  check_device "$SRC"
  if $EXISTING; then
    [[ -f "$OUT" && ! -L "$OUT" ]] || die 'Vorhandenes reguläres Rohimage fehlt.'
    [[ ! "$OUT" -ef "$SRC" ]] || die 'Quelle und Image müssen verschieden sein.'
  else
    [[ ! -e "$OUT" && ! -L "$OUT" ]] || die "Datei existiert bereits: $OUT"
  fi
  for path in "$OUT.sha256" "$OUT.bytes"; do
    [[ ! -e "$path" && ! -L "$path" ]] || die "Datei existiert bereits: $path (für geprüfte Rohkopien --resume verwenden)."
  done
fi
for path in "$OUT.gz" "$OUT.gz.sha256" "$OUT.gz.bytes"; do
  [[ ! -e "$path" && ! -L "$path" ]] || die "Datei existiert bereits: $path"
done
if $SHRINK; then command -v pishrink.sh >/dev/null || die 'pishrink.sh fehlt.'; fi
if $RESUME; then
  echo '==> Prüfe gespeicherte Rohkopie ...'
  (cd "$(dirname "$OUT")"; sha256sum -c -- "$(basename "$OUT").sha256")
  bytes=$(cat "$OUT.bytes")
  [[ "$bytes" =~ ^[1-9][0-9]*$ && $(stat -c %s "$OUT") == "$bytes" ]] || die 'Größe der Rohkopie stimmt nicht.'
  check_image "$OUT"
  RAW_READY=true
else
  bytes=$(blockdev --getsize64 "$SRC")
fi
check_space() {
  local phase=$1 payload=$2 available required
  available=$(df -B1 --output=avail "$(dirname "$OUT")" | tail -n 1)
  available=${available//[[:space:]]/}
  [[ "$available" =~ ^[0-9]+$ ]] || die 'Freier Speicher nicht ermittelbar.'
  required=$((payload + 1073741824))
  echo "==> Platzprüfung ($phase): $available Bytes frei, $required Bytes benötigt."
  ((available >= required)) || die "Zu wenig freier Platz für $phase: $required Bytes benötigt."
}
# Ein GiB, nicht eine Milliarde Bytes. GNU-dd-Offsets in Bytes, auch beim Reststück.
CHUNK_BYTES=1073741824
process_chunks() {
  local image=$1 copy=$2 offset=0 length part=0
  while ((offset < bytes)); do
    length=$((bytes - offset))
    ((length <= CHUNK_BYTES)) || length=$CHUNK_BYTES
    part=$((part + 1))
    check_device "$SRC"
    echo "==> Abschnitt $part: Bytes $offset bis $((offset + length - 1))"
    if $copy; then
      dd if="$SRC" of="$image" bs=4M skip="$offset" seek="$offset" count="$length" \
        iflag=direct,fullblock,skip_bytes,count_bytes oflag=seek_bytes \
        conv=notrunc,fsync status=progress
      [[ $(stat -c %s "$image") == "$((offset + length))" ]] || die 'Unvollständiger Abschnitt.'
    fi
    check_device "$SRC"
    # Image vor dem erneuten Lesen synchronisieren und Cache möglichst verwerfen.
    # Quelle explizit direkt lesen, damit der Vergleich nicht den Lesecache prüft.
    dd if="$image" of=/dev/null count=0 iflag=nocache status=none
    if ! dd if="$SRC" bs=4M skip="$offset" count="$length" \
        iflag=direct,fullblock,skip_bytes,count_bytes status=none |
        cmp -n "$length" -i "0:$offset" -- - "$image"; then
      die "Abschnitt $part (ab Byte $offset) nicht identisch oder Lesefehler. Keine Freigabe, kein Shrink."
    fi
    echo "==> Abschnitt $part stimmt überein."
    offset=$((offset + length))
  done
  check_device "$SRC"
}
write_metadata() {
  local file=$1 size=$2
  printf '%s\n' "$size" > "$file.bytes"
  (cd "$(dirname "$file")"; sha256sum -- "$(basename "$file")" > "$(basename "$file").sha256")
}
if ! $RESUME; then
  if $EXISTING; then
    [[ $(stat -c %s "$OUT") == "$bytes" ]] || die 'Rettungsimage und Quellgerät sind nicht gleich groß.'
    echo '==> Prüfe vorhandenes Image; Image und ddrescue-Map bleiben unverändert ...'
    process_chunks "$OUT" false
    check_image "$OUT"
  else
    check_space "Rohkopie" "$bytes"
    WORK=$(mktemp "${OUT}.partial.XXXXXX")
    process_chunks "$WORK" true
    check_image "$WORK"
    mv -- "$WORK" "$OUT"; WORK=''
  fi
  write_metadata "$OUT" "$bytes"
  sync
  RAW_READY=true
  echo "==> Geprüfte Rohkopie dauerhaft gespeichert: $OUT"
fi
RESULT=$OUT
if $SHRINK; then
  check_space "Shrink-Arbeitskopie" "$(stat -c %s "$OUT")"
  WORK=$(mktemp "${OUT}.shrink.XXXXXX")
  echo '==> Erstelle Arbeitskopie für PiShrink (Reflink, wenn unterstützt) ...'
  cp --reflink=auto --sparse=always -- "$OUT" "$WORK"
  echo '==> Verkleinere Arbeitskopie ...'
  pishrink.sh "$WORK"
  check_image "$WORK"
  bytes=$(stat -c %s "$WORK")
  check_space "Komprimierung" "$((bytes + (bytes + 999) / 1000 + 1048576))"
  echo '==> Komprimiere und prüfe gzip ...'
  PACKED=$(mktemp "${OUT}.gz.partial.XXXXXX")
  gzip -c -- "$WORK" > "$PACKED"
  gzip -t -- "$PACKED"
  gzip -dc -- "$PACKED" | cmp -- "$WORK" -
  printf '%s\n' "$bytes" > "$PACKED.bytes"
  # Metadaten für den endgültigen Namen erzeugen, bevor Ergebnisse publiziert werden.
  digest=$(sha256sum < "$PACKED")
  digest=${digest%% *}
  printf '%s  %s\n' "$digest" "$(basename "$OUT").gz" > "$PACKED.sha256"
  mv -- "$PACKED.bytes" "$OUT.gz.bytes"
  mv -- "$PACKED.sha256" "$OUT.gz.sha256"
  mv -- "$PACKED" "$OUT.gz"; PACKED=''
  RESULT="$OUT.gz"
fi
sync
echo "==> Geprüftes Image: $RESULT"
echo "==> Mindestgröße der Zielkarte: $bytes Bytes (unkomprimiert)."
echo 'Nach dem Schreiben: --verify /dev/sdY -o IMAGE vor dem ersten Boot ausführen.'
