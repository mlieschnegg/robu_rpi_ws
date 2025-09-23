# Datei: rpi_detect.sh
# Nutzung: source ./rpi_detect.sh

# interne Hilfsfunktion: Modell-String aus Device Tree oder Fallbacks lesen
_rpi_read_model() {
  local f
  for f in /sys/firmware/devicetree/base/model /proc/device-tree/model; do
    if [[ -r "$f" ]]; then
      # Device-Tree-Strings sind null-terminiert -> NULs entfernen
      tr -d '\0' < "$f"
      return 0
    fi
  done
  # Fallback über /proc/cpuinfo
  if grep -qi '^model\s*:' /proc/cpuinfo 2>/dev/null; then
    grep -i '^model\s*:' /proc/cpuinfo | head -n1 | cut -d: -f2- | sed 's/^[[:space:]]*//'
    return 0
  fi
  return 1
}

# Gibt 0 zurück, wenn Gerät ein Raspberry Pi ist, sonst 1
is_raspberry_pi() {
  local model
  model="$(_rpi_read_model)" || return 1
  [[ "$model" == *"Raspberry Pi"* ]]
}

# Gibt 0 zurück, wenn es ein Compute Module ist, sonst 1
is_compute_module() {
  local model
  model="$(_rpi_read_model)" || return 1
  # Offizielle Strings enthalten "Compute Module"
  [[ "$model" == *"Compute Module"* ]]
}

# Gibt das vollständige Modell als String aus (z.B. "Raspberry Pi 4 Model B Rev 1.5")
get_rpi_model() {
  _rpi_read_model
}

# Liest die "Revision" aus /proc/cpuinfo (falls vorhanden)
get_rpi_revision() {
  if [[ -r /proc/cpuinfo ]]; then
    grep -i '^Revision\s*:' /proc/cpuinfo | tail -n1 | awk -F': ' '{print $2}'
  fi
}

# Optional: human-readable Typ (echo "none" | "pi" | "compute-module")
get_rpi_type() {
  if is_raspberry_pi; then
    if is_compute_module; then
      echo "compute-module"
    else
      echo "pi"
    fi
  else
    echo "none"
  fi
}
