#!/usr/bin/env bash
# Vorbereitung des Systems vor dem SD-Backup
set -euo pipefail


# chmod +x prepare_pi_for_backup.sh
# sudo ./prepare_pi_for_backup.sh
# # oder ohne Shutdown:
# sudo ./prepare_pi_for_backup.sh --no-shutdown


usage() {
  cat <<'USAGE'
Usage: sudo ./prepare_pi_for_backup.sh [--no-shutdown]
Standardmäßig fährt das Skript am Ende sauber herunter.
Mit --no-shutdown wird NICHT heruntergefahren (nur Sync).
USAGE
}

SHUTDOWN=true
if [[ "${1:-}" == "--no-shutdown" ]]; then
  SHUTDOWN=false
elif [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage; exit 0
fi

if [[ $EUID -ne 0 ]]; then
  echo "Bitte mit sudo als root ausführen."; exit 1
fi

rm -rf ~/snap/firefox/
rm -rf ~/.cache/vscode-cpptools/*
rm -rf ~/.cache/pip/*
rm -rf ~/Downloads/*
rm -rf ~/.python_history
rm -rf ~/.bash_history
rm -rf ~/.local/share/Trash/{files,info}/*
rm -f ~/.ssh/config
rm -f ~/.ssh/authorized_keys
rm -f ~/.ssh/known_hosts
rm -f ~/.ssh/known_hosts.old

snap list --all | awk '/disabled/{print $1, $3}' | while read snapname revision; do sudo snap remove --purge "$snapname" --revision="$revision"; done


echo "==> Paketlisten aktualisieren ..."
apt update

echo "==> Upgrades einspielen ..."
DEBIAN_FRONTEND=noninteractive apt -y upgrade

echo "==> Überflüssige Pakete entfernen & Cache säubern ..."
apt -y autoremove
apt -y clean

echo "==> Paketkonsistenz prüfen/reparieren ..."
dpkg --configure -a
apt -y -f install

echo "==> (Optional) fehlende .list-Dateien reparieren ..."
# Wenn Pakete nicht existieren, nicht fehlschlagen:
apt-get -y --reinstall install powermgmt-base procps || true

# Optionaler Integritäts-Check, nur wenn debsums verfügbar:
if command -v debsums >/dev/null 2>&1; then
  echo "==> debsums-Check (nur Ausgabe, Script bricht nicht ab) ..."
  debsums -s || true
else
  echo "Hinweis: 'debsums' nicht installiert (optional). Installiere mit: sudo apt-get install -y debsums"
fi

echo "==> Daten flushen ..."
sync

if $SHUTDOWN; then
  echo "==> Sauberes Herunterfahren in 5 Sekunden (Strg+C zum Abbrechen) ..."
  sleep 5
  shutdown -h now
else
  echo "FERTIG. Du kannst die SD-Karte jetzt sicher entfernen (System NICHT heruntergefahren)."
fi


