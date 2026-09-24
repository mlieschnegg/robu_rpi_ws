#!/usr/bin/env bash
# Vorbereitung des Systems vor dem SD-Backup
set -euo pipefail


# chmod +x prepare_pi_for_backup.sh
# ./prepare_pi_for_backup.sh
# # oder ohne Shutdown:
# ./prepare_pi_for_backup.sh --no-shutdown


usage() {
  cat <<'USAGE'
Usage: ./prepare_pi_for_backup.sh [--no-shutdown]
Standardmäßig fährt das Skript am Ende sauber herunter.
Mit --no-shutdown wird NICHT heruntergefahren (nur Sync).
Als normaler Benutzer starten; Systemaktionen verwenden intern sudo.
USAGE
}

SHUTDOWN=true
while (($#)); do
  case "$1" in
    --no-shutdown) SHUTDOWN=false; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unbekannte Option: $1" >&2; usage; exit 1;;
  esac
done

if [[ $EUID -eq 0 ]]; then
  echo "Bitte als normaler Benutzer ohne vorangestelltes sudo starten." >&2
  exit 1
fi
[[ -n "${HOME:-}" && "$HOME" == /* && "$HOME" != / && -d "$HOME" && -O "$HOME" ]] || {
  echo "Kein gültiges eigenes Home-Verzeichnis: ${HOME:-<leer>}" >&2; exit 1;
}
# Vor der ersten Löschung sicherstellen, dass Systemaktionen erlaubt sind.
command -v sudo >/dev/null || { echo "sudo fehlt." >&2; exit 1; }
sudo -v


# Nicht nur das aktive Konto prüfen: gh kann mehrere Konten speichern.
# Umgebungs-Tokens ausblenden, damit gespeicherte Anmeldungen sichtbar bleiben.
fail_github() { echo "GitHub-Bereinigung fehlgeschlagen: $*" >&2; exit 1; }
for tool in gh python3 git; do
  command -v "$tool" >/dev/null || fail_github "$tool fehlt; bitte installieren und erneut starten."
done
github_cli() {
  env -u GH_TOKEN -u GITHUB_TOKEN GH_PROMPT_DISABLED=1 gh "$@"
}
github_accounts() {
  github_cli auth status --hostname github.com --json hosts | python3 -c '
import json, sys
try:
    hosts = json.load(sys.stdin)["hosts"]
    if not isinstance(hosts, dict):
        raise ValueError()
    accounts = hosts.get("github.com", [])
    if not isinstance(accounts, list):
        raise ValueError()
    for account in accounts:
        login = account.get("login")
        if not isinstance(login, str) or not login or "\n" in login:
            raise ValueError()
        print(login)
except (ValueError, KeyError, TypeError, AttributeError):
    sys.exit("GitHub-Konten konnten nicht eindeutig ermittelt werden.")
'
}
echo "==> Prüfe GitHub-Anmeldung von mlieschnegg ..."
accounts=$(github_accounts) || fail_github 'Kontenprüfung nicht möglich (aktuelle gh-Version erforderlich).'
while IFS= read -r account; do
  if [[ "${account,,}" == mlieschnegg ]]; then
    echo "==> Melde GitHub-Konto $account lokal ab ..."
    github_cli auth logout --hostname github.com --user "$account" </dev/null || fail_github 'Abmeldung nicht möglich.'
  fi
done <<< "$accounts"
accounts=$(github_accounts) || fail_github 'Abmeldung konnte nicht überprüft werden.'
while IFS= read -r account; do
  [[ "${account,,}" != mlieschnegg ]] || fail_github 'Konto ist weiterhin gespeichert.'
done <<< "$accounts"
# Auch konfigurierte Git-HTTPS-Credential-Helper zum Löschen auffordern.
# Keine Tokens abrufen oder ausgeben. Im bisherigen Arbeitsverzeichnis bleiben,
# damit auch die dortige Repository-Konfiguration berücksichtigt wird.
printf 'protocol=https\nhost=github.com\nusername=mlieschnegg\n\n' |
  GIT_TERMINAL_PROMPT=0 git credential reject || fail_github 'Git-Zugangsdaten konnten nicht entfernt werden.'
if [[ -n "${GH_TOKEN:-}" || -n "${GITHUB_TOKEN:-}" ]]; then
  fail_github 'GH_TOKEN/GITHUB_TOKEN gesetzt. Token aus Shell und dauerhafter Konfiguration entfernen und erneut starten; unset im Skript reicht nicht.'
fi
echo "==> mlieschnegg ist in der GitHub CLI dieses Benutzers abgemeldet."
echo "Andere Zugänge (Browser, VS Code, SSH-Schlüssel, andere Benutzer) separat entfernen."

rm -rf ~/snap/firefox/
rm -rf ~/.cache/vscode-cpptools/*
rm -rf ~/.cache/pip/*
rm -rf ~/Downloads/*
rm -rf ~/.python_history
# Offene Bash-Sitzungen können die Datei beim Beenden erneut schreiben.
# Den Shell-Speicher kann nur die jeweilige interaktive Shell leeren.
rm -f -- "$HOME/.bash_history"
echo "History vor dem Herunterfahren in JEDER offenen Bash löschen:"
echo "  history -c; history -w; unset HISTFILE"
echo "Dies muss in der interaktiven Shell geschehen, nicht in diesem Skript."
rm -rf ~/.local/share/Trash/{files,info}/*
rm -f ~/.ssh/config
rm -f ~/.ssh/authorized_keys
rm -f ~/.ssh/known_hosts
rm -f ~/.ssh/known_hosts.old

if command -v snap >/dev/null 2>&1; then
  sudo snap list --all | awk '/disabled/{print $1, $3}' | while read -r snapname revision; do sudo snap remove --purge "$snapname" --revision="$revision"; done
fi


echo "==> Paketlisten aktualisieren ..."
sudo apt update

echo "==> Upgrades einspielen ..."
sudo env DEBIAN_FRONTEND=noninteractive apt -y upgrade

echo "==> Überflüssige Pakete entfernen & Cache säubern ..."
sudo apt -y autoremove
sudo apt -y clean

echo "==> Paketkonsistenz prüfen/reparieren ..."
sudo dpkg --configure -a
sudo apt -y -f install

echo "==> (Optional) fehlende .list-Dateien reparieren ..."
# Wenn Pakete nicht existieren, nicht fehlschlagen:
sudo apt-get -y --reinstall install powermgmt-base procps || true

# Optionaler Integritäts-Check, nur wenn debsums verfügbar:
if command -v debsums >/dev/null 2>&1; then
  echo "==> debsums-Check (nur Ausgabe, Script bricht nicht ab) ..."
  sudo debsums -s || true
else
  echo "Hinweis: 'debsums' nicht installiert (optional). Installiere mit: sudo apt-get install -y debsums"
fi

echo "==> Daten flushen ..."
sync

if $SHUTDOWN; then
  echo "==> Sauberes Herunterfahren in 5 Sekunden (Strg+C zum Abbrechen) ..."
  sleep 5
  sudo shutdown -h now
else
  echo "FERTIG. System läuft noch! Erst vollständig herunterfahren, dann SD-Karte entfernen."
fi


