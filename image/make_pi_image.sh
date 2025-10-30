#!/usr/bin/env bash
# Sichere SD-Karte per dd -> Image, optional pishrink (-z für gzip), FS-Checks und Checksumme
set -euo pipefail

# Beispielaufruf:
# chmod +x make_pi_image.sh
# sudo ./make_pi_image.sh -d /dev/sdb -o raspi_master.img --shrink

# Nach dem Erstellen des Images (ggf. mit --shrink) kann die Integrität mit folgendem Befehl geprüft werden:
# sha256sum -c raspi_master.img.gz.sha256


usage() {
  cat <<'USAGE'
Usage: sudo ./make_pi_image.sh -d /dev/sdX -o raspi_master.img [--shrink]
  -d  Quellgerät der SD-Karte (z.B. /dev/sdb)  *GANZES Gerät, nicht die Partition!*
  -o  Ziel-Image-Datei (z.B. raspi_master.img)
  --shrink  Image nach dem Erstellen mit pishrink verkleinern und gzip-komprimieren (-z)
Beispiel:
  sudo ./make_pi_image.sh -d /dev/sdb -o raspi_master.img --shrink
USAGE
}

SRC=""
OUT=""
SHRINK=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d) SRC="${2:-}"; shift 2;;
    -o) OUT="${2:-}"; shift 2;;
    --shrink) SHRINK=true; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unbekannte Option: $1"; usage; exit 1;;
  esac
done

if [[ -z "$SRC" || -z "$OUT" ]]; then
  usage; exit 1
fi

if [[ $EUID -ne 0 ]]; then
  echo "Bitte mit sudo als root ausführen."; exit 1
fi

if [[ ! -b "$SRC" ]]; then
  echo "Quellgerät $SRC existiert nicht oder ist kein Blockgerät."; exit 1
fi

if lsblk -no MOUNTPOINT "$SRC" | grep -q '/'; then
  echo "Fehler: $SRC ist gemountet. Bitte aushängen und erneut versuchen."; exit 1
fi

echo "==> Erstelle Roh-Image mit dd ..."
dd if="$SRC" of="$OUT" bs=4M status=progress iflag=fullblock conv=fsync

echo "==> Richte Loop-Device ein ..."
LOOP=$(losetup -Pf --show "$OUT")

cleanup() {
  set +e
  sync
  losetup -d "$LOOP" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "==> Prüfe Dateisystem der Root-Partition (typischerweise p2) ..."
if [[ -b "${LOOP}p2" ]]; then
  e2fsck -f -p "${LOOP}p2" || {
    echo "e2fsck meldete Fehler. Starte interaktiven Fix ..."
    e2fsck -f -y "${LOOP}p2"
  }
else
  echo "Hinweis: ${LOOP}p2 nicht gefunden. Prüfe mit 'fdisk -l $OUT'."
fi

if $SHRINK; then
  echo "==> Verkleinere und komprimiere Image mit pishrink (-z) ..."
  losetup -d "$LOOP"
  trap - EXIT

  if ! command -v pishrink.sh >/dev/null 2>&1; then
    echo "pishrink.sh nicht gefunden. Installiere/binde es ein und starte erneut."; exit 1
  fi

  pishrink.sh -z "$OUT"

  # Der neue Dateiname enthält jetzt .gz am Ende
  OUT="${OUT}.gz"

  echo "==> Prüfe nach dem Shrink, ob gzip-Datei existiert ..."
  if [[ ! -f "$OUT" ]]; then
    echo "Fehler: komprimierte Datei wurde nicht erstellt!"; exit 1
  fi

  echo "==> Erzeuge SHA256-Prüfsumme der komprimierten Datei ..."
  sha256sum "$OUT" | tee "${OUT}.sha256"
else
  echo "==> Erzeuge SHA256-Prüfsumme ..."
  sha256sum "$OUT" | tee "${OUT}.sha256"
fi

echo "==> Fertig! Ergebnis:"
ls -lh "$OUT" "${OUT}.sha256"
