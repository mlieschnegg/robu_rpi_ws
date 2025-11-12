import subprocess

def run_cmd(cmd):
    """Hilfsfunktion: führt ein Kommando aus und gibt den Text zurück."""
    try:
        output = subprocess.check_output(cmd).decode().strip()
        return output if output else ""
    except Exception:
        return ""

def get_ip_address():
    """Ermittelt die erste IP-Adresse."""
    output = run_cmd(['hostname', '-I'])
    return output.split()[0] if output != "" else ""

def get_interface():
    """Ermittelt das aktive Netzwerkinterface (z.B. wlan0 oder eth0)."""
    # 'ip route' liefert z.B.: "default via 192.168.0.1 dev wlan0 ..."
    output = run_cmd(['ip', 'route'])
    for part in output.split():
        if part == 'dev':
            idx = output.split().index(part)
            return output.split()[idx + 1]
    return "Unknown"

def get_ssid(interface):
    """Ermittelt SSID, falls WLAN — sonst 'LAN'."""
    if interface.startswith('wl'):  # wlan0, wlp2s0 etc.
        ssid = run_cmd(['iwgetid', '-r'])
        return ssid if ssid != "" else ""
    else:
        return "LAN"

def get_current_wifi_signal() -> tuple[str, int]:
    # -t = terse, -f = Felder
    cmd = ["nmcli", "-t", "-f", "active,ssid,signal", "dev", "wifi"]
    out = subprocess.check_output(cmd, text=True)

    for line in out.splitlines():
        # erwartetes Format: yes:<ssid>:<signal>
        parts = line.split(":")
        if len(parts) >= 3 and parts[0] == "yes":
            ssid = parts[1]
            signal = parts[2]  # in Prozent
            return ssid, int(signal)

    return "", 0