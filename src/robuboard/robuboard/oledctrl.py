#!/usr/bin/env python3
import os
import subprocess
from smbus import SMBus
from rpi.oled096_ssd1306 import SSD1306

# =======================
#   Hilfsfunktionen
# =======================

def run_cmd(cmd):
    """Hilfsfunktion: führt ein Kommando aus und gibt den Text zurück."""
    try:
        output = subprocess.check_output(cmd).decode().strip()
        return output if output else "n/a"
    except Exception:
        return "n/a"

def get_ip_address():
    """Ermittelt die erste IP-Adresse."""
    output = run_cmd(['hostname', '-I'])
    return output.split()[0] if output != "n/a" else "No IP"

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
        return ssid if ssid != "n/a" else "No SSID"
    else:
        return "LAN"

def init_display():
    """Initialisiert das OLED-Display."""
    i2cbus = SMBus(1)
    return SSD1306(i2cbus)

def draw_info(oled, hostname, ip, interface, ssid):
    """Zeichnet die Infos auf dem Display."""
    oled.canvas.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
    oled.canvas.rectangle((0, 0, oled.width-1, oled.height-1), outline=1, fill=0)

    oled.canvas.text((5, 5),  f"{hostname}", fill=1)
    oled.canvas.text((5, 20), f"IP: {ip}", fill=1)
    oled.canvas.text((5, 35), f"Pwd: #robotic", fill=1)
    oled.canvas.text((5, 50), f"Netz: {ssid}", fill=1)

    oled.display()

# =======================
#   Hauptprogramm
# =======================

if __name__ == "__main__":
    hostname = os.uname().nodename
    oled = init_display()

    interface = get_interface()
    ip_address = get_ip_address()
    ssid = get_ssid(interface)

    draw_info(oled, hostname, ip_address, interface, ssid)
