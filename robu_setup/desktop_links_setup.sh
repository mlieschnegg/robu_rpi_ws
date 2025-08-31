#!/usr/bin/env bash
set -euo pipefail

# Ziel: KDE-Desktop-Ordner ermitteln
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
mkdir -p "$DESKTOP_DIR"

have() { command -v "$1" >/dev/null 2>&1; }

# Ermittelt passenden Exec-Befehl für App: native > flatpak > snap
resolve_exec() {
  local app="$1" ; shift
  local -a candidates=("$@")

  # 1) Direkte Binärdatei aus $PATH bevorzugen
  for c in "${candidates[@]}"; do
    if have "$c"; then
      echo "$c"
      return 0
    fi
  done

  # 2) Flatpak?
  if have flatpak; then
    case "$app" in
      blender)
        if flatpak list --app --columns=application | grep -q '^org.blender.Blender$'; then
          echo "flatpak run org.blender.Blender"; return 0
        fi
        ;;
      code)
        # VS Code / Codium
        if flatpak list --app --columns=application | grep -q '^com.visualstudio.code$'; then
          echo "flatpak run com.visualstudio.code"; return 0
        elif flatpak list --app --columns=application | grep -q '^com.vscodium.codium$'; then
          echo "flatpak run com.vscodium.codium"; return 0
        fi
        ;;
      gimp)
        if flatpak list --app --columns=application | grep -q '^org.gimp.GIMP$'; then
          echo "flatpak run org.gimp.GIMP"; return 0
        fi
        ;;
      netbeans)
        if flatpak list --app --columns=application | grep -q '^org.apache.netbeans$'; then
          echo "flatpak run org.apache.netbeans"; return 0
        fi
        ;;
    esac
  fi

  # 3) Snap?
  if have snap; then
    case "$app" in
      blender)
        if snap list 2>/dev/null | awk '{print $1}' | grep -qx blender; then
          echo "snap run blender"; return 0
        fi
        ;;
      code)
        if snap list 2>/dev/null | awk '{print $1}' | grep -qx code; then
          echo "snap run code"; return 0
        elif snap list 2>/dev/null | awk '{print $1}' | grep -qx codium; then
          echo "snap run codium"; return 0
        fi
        ;;
      gimp)
        if snap list 2>/dev/null | awk '{print $1}' | grep -qx gimp; then
          echo "snap run gimp"; return 0
        fi
        ;;
      netbeans)
        if snap list 2>/dev/null | awk '{print $1}' | grep -qx netbeans; then
          echo "snap run netbeans"; return 0
        fi
        ;;
    esac
  fi

  return 1
}

# .desktop-Datei schreiben
make_shortcut() {
  local name="$1" exec_cmd="$2" icon_name="$3" filename="$4"
  local desktop_path="$DESKTOP_DIR/$filename"

  cat > "$desktop_path" <<EOF
[Desktop Entry]
Type=Application
Name=$name
Exec=$exec_cmd
Icon=$icon_name
Terminal=false
Categories=Development;Graphics;IDE;Utility;
StartupNotify=true
EOF

  chmod +x "$desktop_path"
  echo "✔ $name → $desktop_path"
}

# App-Definitionen: Name | App-Key | Kandidaten (PATH) | Icon-Namen (1. bevorzugt)
declare -A NAMES=(
  [blender]="Blender"
  [code]="Visual Studio Code"
  [gimp]="GIMP"
  [netbeans]="Apache NetBeans"
)

declare -A CANDIDATES=(
  [blender]="blender"
  [code]="code code-insiders codium"
  [gimp]="gimp"
  [netbeans]="netbeans nbexec"
)

declare -A ICONS=(
  [blender]="blender org.blender.Blender"
  [code]="visual-studio-code code com.visualstudio.code codium com.vscodium.codium"
  [gimp]="gimp org.gimp.GIMP"
  [netbeans]="netbeans org.apache.netbeans"
)

create_for() {
  local key="$1"
  # Exec
  IFS=' ' read -r -a cand <<< "${CANDIDATES[$key]}"
  if ! exec_cmd="$(resolve_exec "$key" "${cand[@]}")"; then
    echo "⚠ ${NAMES[$key]} nicht gefunden (weder native, noch Flatpak/Snap). Überspringe."
    return 0
  fi
  # Icon (erster Eintrag genügt, Icon-Theme löst auf)
  local icon="${ICONS[$key]}"
  icon="${icon%% *}"

  # Dateiname
  local file="${NAMES[$key]// /_}.desktop"
  make_shortcut "${NAMES[$key]}" "$exec_cmd" "$icon" "$file"
}

echo "Erzeuge Desktop-Verknüpfungen in: $DESKTOP_DIR"
create_for blender
create_for code
create_for gimp
create_for netbeans

echo "Fertig. Falls die Icons nicht sofort sichtbar sind, den Desktop einmal neu laden."
