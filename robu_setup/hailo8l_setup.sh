#!/bin/bash
# Hailo-8L im M.2-Slot des Robuboard mit Compute Module 5 einrichten.
# Auf dem Robuboard ausführen (sudo wird bei Bedarf automatisch verwendet):
#   bash ~/work/.robu/robu_setup/hailo8l_setup.sh
# Nach dem anschließenden Neustart prüfen:
#   bash ~/work/.robu/robu_setup/hailo8l_setup.sh --check
#
# Installiert PCIe-Kerneltreiber und Firmware aus der getesteten Mitschrift.
# HailoRT-Bibliothek, Python-Bindings und Modelle werden separat eingerichtet;
# die HailoRT-Version muss zum Treiber passen (hier 4.24.0).
# Quellen:
#   https://github.com/hailo-ai/hailort-drivers/tree/v4.24.0
#   https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#pcie-gen-3-0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=rpi_detect.sh
source "$SCRIPT_DIR/rpi_detect.sh"

# Festes Release statt des beweglichen Branches hailo8: Treiber und Firmware
# bleiben auch bei einer späteren Installation auf demselben Versionsstand.
readonly HAILO_VERSION="4.24.0"
readonly HAILO_MODULE="hailo_pci"
readonly HAILO_REPOSITORY="https://github.com/hailo-ai/hailort-drivers.git"
readonly BOOT_CONFIG="/boot/firmware/config.txt"
SUDO=()
BUILD_DIR=""

log() { printf '[Hailo-8L] %s\n' "$*"; }
fail() { log "FEHLER: $*" >&2; exit 1; }

cleanup() {
    # Nur das von diesem Lauf erzeugte Verzeichnis löschen. Ein vorhandener
    # Checkout unter ~/work/hailort-drivers wird weder benutzt noch entfernt.
    if [[ -n "$BUILD_DIR" ]]; then
        rm -rf -- "$BUILD_DIR"
    fi
}

require_cm5() {
    local model
    model="$(get_rpi_model)" || fail "Kein Raspberry Pi erkannt. Auf dem Robuboard ausführen."
    [[ "$model" == *"Raspberry Pi Compute Module 5"* ]] ||
        fail "Dieses Setup ist für das Robuboard mit CM5 vorgesehen. Erkannt: $model"
}

prepare_sudo() {
    if (( EUID != 0 )); then
        SUDO=(sudo)
        sudo -v
    fi
}

render_boot_config() {
    # Einstellungen am Ende des jeweiligen [cm5]-Abschnitts setzen, damit
    # vorherige Einzelwerte überschrieben werden. Andere Abschnitte erhalten.
    # Das erneute Ausführen erzeugt weder doppelte Einträge noch neue Blöcke.
    awk '
        function settings() {
            print "# Hailo-8L im M.2-Slot des Robuboard: PCIe mit Gen3 aktivieren"
            print "dtparam=pciex1"
            print "dtparam=pciex1_gen=3"
        }
        {
            line = $0
            sub(/[[:space:]]*#.*/, "", line)
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", line)
            if (line ~ /^\[[^]]+\]$/) {
                if (cm5) settings()
                cm5 = (line == "[cm5]")
                if (cm5) found = 1
            }
            if (cm5 && ($0 ~ /^# Hailo-8L im M.2-Slot des Robuboard:/ ||
                line ~ /^dtparam[[:space:]]*=[[:space:]]*pciex1(=(on|off))?$/ ||
                line ~ /^dtparam[[:space:]]*=[[:space:]]*pciex1_gen=[0-9]+$/)) next
            print
        }
        END {
            if (cm5) settings()
            if (!found) {
                print "\n[cm5]"
                settings()
                print "[all]"
            }
        }
    ' "$1"
}

configure_pcie() {
    local updated backup
    updated="$BUILD_DIR/config.txt"
    render_boot_config "$BOOT_CONFIG" > "$updated"
    if cmp -s "$BOOT_CONFIG" "$updated"; then
        log "PCIe Gen3 ist in [cm5] bereits konfiguriert."
        return
    fi
    backup="${BOOT_CONFIG}.before-hailo8l.$(date +%Y%m%d-%H%M%S).$$"
    "${SUDO[@]}" cp -a "$BOOT_CONFIG" "$backup"
    # Inhalt ersetzen und die Berechtigungen der vorhandenen Datei erhalten.
    "${SUDO[@]}" tee "$BOOT_CONFIG" < "$updated" > /dev/null
    log "PCIe Gen3 in [cm5] aktiviert. Sicherung: $backup"
}

write_dkms_config() {
    printf 'PACKAGE_NAME="%s"\nPACKAGE_VERSION="%s"\n' "$HAILO_MODULE" "$HAILO_VERSION"
    # Variablen erst durch DKMS auswerten lassen: Nach einem Kernel-Update muss
    # gegen dessen Header gebaut werden, nicht gegen den gerade laufenden Kernel.
    cat <<'EOF'
BUILT_MODULE_NAME[0]="hailo_pci"
BUILT_MODULE_LOCATION[0]="linux/pcie"
DEST_MODULE_LOCATION[0]="/updates/dkms"
AUTOINSTALL="yes"
MAKE[0]="make -C \"${kernel_source_dir}\" M=\"${dkms_tree}/${PACKAGE_NAME}/${PACKAGE_VERSION}/build/linux/pcie\" modules"
CLEAN="make -C \"${kernel_source_dir}\" M=\"${dkms_tree}/${PACKAGE_NAME}/${PACKAGE_VERSION}/build/linux/pcie\" clean"
EOF
}

install_driver() {
    local kernel="$1" source_dir registered
    source_dir="/usr/src/${HAILO_MODULE}-${HAILO_VERSION}"

    git clone --depth 1 --branch "v${HAILO_VERSION}" \
        "$HAILO_REPOSITORY" "$BUILD_DIR/hailort-drivers"
    (
        cd "$BUILD_DIR/hailort-drivers"
        bash ./download_firmware.sh
    )
    [[ -s "$BUILD_DIR/hailort-drivers/hailo8_fw.${HAILO_VERSION}.bin" ]] ||
        fail "Die heruntergeladene Firmware fehlt oder ist leer."
    "${SUDO[@]}" install -D -m 644 \
        "$BUILD_DIR/hailort-drivers/hailo8_fw.${HAILO_VERSION}.bin" \
        /lib/firmware/hailo/hailo8_fw.bin

    # DKMS behält den Quellcode dauerhaft in /usr/src. Deshalb darf das temporäre
    # Git-Repository nach dem Setup entfernt werden. Registrierte Quellen bleiben
    # bei erneutem Ausführen erhalten, auch wenn andere Kernel sie bereits nutzen.
    registered="$("${SUDO[@]}" dkms status -m "$HAILO_MODULE" -v "$HAILO_VERSION")"
    if [[ -z "$registered" ]]; then
        write_dkms_config > "$BUILD_DIR/dkms.conf"
        "${SUDO[@]}" mkdir -p "$source_dir"
        "${SUDO[@]}" cp -a "$BUILD_DIR/hailort-drivers/linux" \
            "$BUILD_DIR/hailort-drivers/common" "$source_dir/"
        "${SUDO[@]}" install -m 644 "$BUILD_DIR/dkms.conf" "$source_dir/dkms.conf"
        "${SUDO[@]}" dkms add -m "$HAILO_MODULE" -v "$HAILO_VERSION"
    fi

    # Der erste Build läuft direkt über DKMS; ein zusätzlicher manueller Build
    # und eine vorläufige Installation von hailo_pci.ko sind dadurch überflüssig.
    "${SUDO[@]}" dkms build -m "$HAILO_MODULE" -v "$HAILO_VERSION" -k "$kernel"
    "${SUDO[@]}" dkms install -m "$HAILO_MODULE" -v "$HAILO_VERSION" -k "$kernel" --force

    # Überbleibsel der manuellen Installation aus der Mitschrift erst entfernen,
    # nachdem DKMS erfolgreich installiert hat. Andere Kernel bleiben unberührt.
    "${SUDO[@]}" rm -f "/lib/modules/$kernel/updates/hailo_pci.ko"
    "${SUDO[@]}" depmod -a "$kernel"

    printf '%s\n' "$HAILO_MODULE" > "$BUILD_DIR/hailo.conf"
    "${SUDO[@]}" install -D -m 644 "$BUILD_DIR/hailo.conf" /etc/modules-load.d/hailo.conf
    # Offizielle Hailo-Regel: Zugriff auf /dev/hailo0 auch ohne sudo ermöglichen.
    "${SUDO[@]}" install -D -m 644 \
        "$BUILD_DIR/hailort-drivers/linux/pcie/51-hailo-udev.rules" \
        /etc/udev/rules.d/51-hailo-udev.rules
    "${SUDO[@]}" udevadm control --reload-rules
    "${SUDO[@]}" udevadm trigger --subsystem-match=hailo_chardev
}

check_hailo() {
    local status=0 dkms_status loaded_version
    log "DKMS-Status für Kernel $(uname -r):"
    dkms_status="$("${SUDO[@]}" dkms status -m "$HAILO_MODULE" -v "$HAILO_VERSION" -k "$(uname -r)")"
    printf '%s\n' "$dkms_status"
    if [[ "$dkms_status" != *": installed"* ]]; then
        log "Treiber ist für den laufenden Kernel nicht über DKMS installiert."
        status=1
    fi
    if ! "${SUDO[@]}" modprobe "$HAILO_MODULE"; then
        log "Treiber konnte nicht geladen werden."
        status=1
    fi
    "${SUDO[@]}" udevadm settle
    if [[ -r /sys/module/hailo_pci/version ]]; then
        loaded_version="$(cat /sys/module/hailo_pci/version)"
        log "Geladene Treiberversion: $loaded_version"
        if [[ "$loaded_version" != "$HAILO_VERSION" ]]; then
            log "Für Version $HAILO_VERSION bitte neu starten."
            status=1
        fi
    fi
    if [[ -c /dev/hailo0 ]]; then
        ls -l /dev/hailo[0-9]*
    else
        log "Kein /dev/hailo0 gefunden. Nach dem Setup neu starten und erneut prüfen."
        status=1
    fi
    log "Letzte Hailo-Kernelmeldungen:"
    "${SUDO[@]}" dmesg | grep -i hailo | tail -25 || true
    return "$status"
}

main() {
    case "${1:-}" in
        -h|--help)
            printf 'Aufruf: bash %s [--check]\n' "$0"
            return ;;
        ''|--check) ;;
        *) fail "Unbekannte Option: $1 (erlaubt: --check, --help)" ;;
    esac
    (( $# <= 1 )) || fail "Zu viele Argumente."
    require_cm5
    prepare_sudo
    if [[ "${1:-}" == --check ]]; then
        check_hailo
        return
    fi
    [[ -f "$BOOT_CONFIG" ]] || fail "$BOOT_CONFIG fehlt."

    local kernel
    kernel="$(uname -r)"
    log "Installiere Treiber und Firmware $HAILO_VERSION für Kernel $kernel."
    "${SUDO[@]}" apt update
    "${SUDO[@]}" apt install -y build-essential dkms git wget ca-certificates "linux-headers-$kernel"
    [[ -f "/lib/modules/$kernel/build/Makefile" ]] ||
        fail "Kernel-Header fehlen. Nach einem Kernel-Update neu starten und Setup erneut ausführen."

    BUILD_DIR="$(mktemp -d "${TMPDIR:-/tmp}/robu-hailo8l.XXXXXX")"
    install_driver "$kernel"
    configure_pcie

    # Ein bereits geladener Treiber wird nicht entladen: Er könnte in Benutzung
    # sein. PCIe-Einstellungen und die installierte Version gelten nach Reboot.
    log "Installation abgeschlossen. Bitte das Robuboard mit sudo reboot neu starten."
    log "Danach testen: bash ~/work/.robu/robu_setup/hailo8l_setup.sh --check"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    # Bei Installationsfehlern abbrechen, damit unvollständige Treiber-/Firmware-
    # Installationen nicht als erfolgreich gemeldet werden.
    set -Eeuo pipefail
    trap 'printf "[Hailo-8L] FEHLER in Zeile %s (Exit %s).\n" "$LINENO" "$?" >&2' ERR
    trap cleanup EXIT
    main "$@"
fi
