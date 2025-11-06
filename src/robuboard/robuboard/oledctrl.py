#!/usr/bin/env python3
import os
import subprocess
from smbus import SMBus
from rpi.oled096_ssd1306 import SSD1306
import time

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

def get_current_wifi_signal():
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

    return None, None

def init_display():
    """Initialisiert das OLED-Display."""
    i2cbus = SMBus(1)
    return SSD1306(i2cbus)

def draw_info_network(oled, hostname:str, ip:str, ssid:str, signal:int):
    """Zeichnet die Infos auf dem Display."""
    oled.canvas.rectangle((0, 0, oled.width-1, oled.height-1), outline=1, fill=0)

    oled.canvas.text((5, 5),  f"HN: {hostname}", fill=1)
    oled.canvas.text((5, 20), f"IP: {ip}", fill=1)
    oled.canvas.text((5, 35), f"SSID: {ssid}", fill=1)
    oled.canvas.text((5, 50), f"Signal: {signal} dbm", fill=1)

    oled.display()

def draw_info_ros(oled, user="robu", pw="robu", id:int=30):
    """Zeichnet die Infos auf dem Display."""
    oled.canvas.rectangle((0, 0, oled.width-1, oled.height-1), outline=1, fill=0)

    oled.canvas.text((5, 5),  f"Linux USER: {user}", fill=1)
    oled.canvas.text((5, 20), f"PW: {user}", fill=1)
    oled.canvas.text((5, 35), f"ROS_DOMAIN_ID: {id}", fill=1)

    oled.display()
    
# =======================
#   Hauptprogramm
# =======================

if __name__ == "__main__":
    hostname = os.uname().nodename
    oled = init_display()

    interface = get_interface()
    ip_address = get_ip_address()
    # ssid = get_ssid(interface)

    ssid, signal = get_current_wifi_signal()
    ros_domain_id = os.getenv("ROS_DOMAIN_ID")

    draw_info_network(oled, hostname, ip_address, ssid,signal)
    time.sleep(1)
    draw_info_ros(oled, id=ros_domain_id)


