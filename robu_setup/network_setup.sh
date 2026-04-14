#!/bin/bash

set -e

IFACE="wlan0"
NETPLAN_FILE="/etc/netplan/90-robu.yaml"

is_desktop() {
    systemctl is-active --quiet NetworkManager
}

configure_networkmanager() {
    echo "Desktop-System erkannt -> konfiguriere WLAN mit NetworkManager"

    for NAME in "robotic" "RoboCup" "ROBU" "S-MISC"; do
        nmcli connection delete "$NAME" 2>/dev/null || true
    done

    nmcli connection add type wifi ifname "$IFACE" con-name "robotic" ssid "robotic"
    nmcli connection modify "robotic" wifi-sec.key-mgmt wpa-psk
    nmcli connection modify "robotic" wifi-sec.psk "#robotic"
    nmcli connection modify "robotic" connection.autoconnect yes
    nmcli connection modify "robotic" connection.autoconnect-priority 10

    nmcli connection add type wifi ifname "$IFACE" con-name "RoboCup" ssid "RoboCup"
    nmcli connection modify "RoboCup" wifi-sec.key-mgmt wpa-psk
    nmcli connection modify "RoboCup" wifi-sec.psk "#robotic"
    nmcli connection modify "RoboCup" connection.autoconnect yes
    nmcli connection modify "RoboCup" connection.autoconnect-priority 20

    nmcli connection add type wifi ifname "$IFACE" con-name "ROBU" ssid "ROBU"
    nmcli connection modify "ROBU" wifi-sec.key-mgmt wpa-psk
    nmcli connection modify "ROBU" wifi-sec.psk "#robotic"
    nmcli connection modify "ROBU" connection.autoconnect yes
    nmcli connection modify "ROBU" connection.autoconnect-priority 30

    nmcli connection add type wifi ifname "$IFACE" con-name "S-MISC" ssid "S-MISC"
    nmcli connection modify "S-MISC" wifi-sec.key-mgmt wpa-psk
    nmcli connection modify "S-MISC" wifi-sec.psk 'sbCLEtuiDMD$#*8C'
    nmcli connection modify "S-MISC" connection.autoconnect yes
    nmcli connection modify "S-MISC" connection.autoconnect-priority 40

    echo "NetworkManager-Konfiguration abgeschlossen."
}

configure_netplan() {
    echo "Server-System erkannt -> konfiguriere WLAN mit Netplan"

    mkdir -p /etc/netplan

    cat > "$NETPLAN_FILE" <<EOF
network:
  version: 2
  renderer: networkd
  wifis:
    $IFACE:
      dhcp4: true
      optional: true
      access-points:
        "S-MISC":
          password: "sbCLEtuiDMD\$#*8C"
        "ROBU":
          password: "#robotic"
        "RoboCup":
          password: "#robotic"
        "robotic":
          password: "#robotic"
EOF

    chmod 600 "$NETPLAN_FILE"
    netplan generate
    netplan apply

    echo "Netplan-Konfiguration abgeschlossen."
    echo "Datei: $NETPLAN_FILE"
}

main() {
    if [[ $EUID -ne 0 ]]; then
        echo "Bitte mit sudo ausführen."
        exit 1
    fi

    if ! ip link show "$IFACE" > /dev/null 2>&1; then
        echo "Netzwerkschnittstelle '$IFACE' nicht gefunden."
        exit 1
    fi

    if is_desktop; then
        configure_networkmanager
    else
        configure_netplan
    fi
}

main "$@"
