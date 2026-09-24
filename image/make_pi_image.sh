#!/usr/bin/env bash
# Offline-Backup einer SD-Karte. Niemals auf dem laufenden Quellsystem ausführen.
set -Eeuo pipefail
export LC_ALL=C
usage() {
  cat <<'USAGE'
Usage: sudo ./make_pi_image.sh -d /dev/sdX -o backup.img [--shrink]
       sudo ./make_pi_image.sh --resume -o backup.img
       sudo ./make_pi_image.sh --verify /dev/sdY -o backup.img[.gz]
  -d          Ganzes, vollständig ausgehängtes Quellgerät
  -o          Neue Image-Datei; bei --verify vorhandenes Image
  --shrink    Rohkopie behalten; Arbeitskopie verkleinern, prüfen, komprimieren
  --resume    Geprüfte Rohkopie wiederverwenden; kein dd, keine SD-Karte nötig
  --verify    Bereits geschriebene Zielkarte VOR dem ersten Boot vergleichen
Es werden FAT-Bootpartition + ext4-Rootpartition (DOS/MBR) unterstützt.
Quelle und Zielkarte werden ausschließlich gelesen. Fehler führen zum Abbruch.
USAGE
}
die() { echo "FEHLER: $*" >&2; exit 1; }
SRC='' OUT='' VERIFY='' LOOP='' WORK='' PACKED='' RAW_READY=false SHRINK=false RESUME=false
while (($#)); do
  case "$1" in
    -d|-o|--verify)
      (($# >= 2)) && [[ -n "$2" ]] || die "Argument für $1 fehlt."
      case "$1" in -d) SRC=$2;; -o) OUT=$2;; --verify) VERIFY=$2;; esac
      shift 2;;
    --shrink) SHRINK=true; shift;;
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
  [[ -z "$SRC" && "$SHRINK" == false && "$RESUME" == false ]] || die '--verify nicht mit -d/--shrink kombinieren.'
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
  [[ -z "$SRC" ]] || die '--resume nicht mit -d kombinieren.'
  [[ -f "$OUT" && -f "$OUT.sha256" && -f "$OUT.bytes" ]] || die 'Geprüfte Rohkopie mit .sha256 und .bytes fehlt.'
else
  [[ -n "$SRC" ]] || die '-d fehlt.'
  SRC=$(realpath "$SRC")
  check_device "$SRC"
  for path in "$OUT" "$OUT.sha256" "$OUT.bytes"; do
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
available=$(df -B1 --output=avail "$(dirname "$OUT")" | tail -n 1)
required=$((bytes + 1073741824))
if $SHRINK; then required=$((3 * bytes + 1073741824)); fi
if $RESUME; then required=$((2 * bytes + 1073741824)); fi
((available >= required)) || die "Zu wenig freier Platz: vorsorglich $required zusätzliche Bytes erforderlich."
write_metadata() {
  local file=$1 size=$2
  printf '%s\n' "$size" > "$file.bytes"
  (cd "$(dirname "$file")"; sha256sum -- "$(basename "$file")" > "$(basename "$file").sha256")
}
if ! $RESUME; then
  WORK=$(mktemp "${OUT}.partial.XXXXXX")
  echo '==> Lese Quelle vollständig ...'
  dd if="$SRC" of="$WORK" bs=4M status=progress iflag=fullblock conv=fsync
  [[ $(stat -c %s "$WORK") == "$bytes" ]] || die 'Unvollständige Kopie.'
  check_device "$SRC"
  blockdev --flushbufs "$SRC"
  echo '==> Zweiter vollständiger Lesedurchlauf: Quelle mit Rohkopie vergleichen ...'
  cmp -n "$bytes" -- "$WORK" "$SRC"
  check_image "$WORK"
  mv -- "$WORK" "$OUT"; WORK=''
  write_metadata "$OUT" "$bytes"
  sync
  RAW_READY=true
  echo "==> Geprüfte Rohkopie dauerhaft gespeichert: $OUT"
fi
RESULT=$OUT
if $SHRINK; then
  WORK=$(mktemp "${OUT}.shrink.XXXXXX")
  echo '==> Erstelle Arbeitskopie für PiShrink (Reflink, wenn unterstützt) ...'
  cp --reflink=auto --sparse=always -- "$OUT" "$WORK"
  echo '==> Verkleinere Arbeitskopie ...'
  pishrink.sh "$WORK"
  check_image "$WORK"
  bytes=$(stat -c %s "$WORK")
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
