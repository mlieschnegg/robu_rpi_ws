#!/bin/bash

# Alle Wifi-Netzwerke löschen
nmcli connection delete robotic RoboCup ROBU S-MISC 2>/dev/null

# WLAN "robotic"
nmcli connection add type wifi ifname wlan0 con-name "robotic" ssid "robotic"
nmcli connection modify robotic wifi-sec.key-mgmt wpa-psk wifi-sec.psk "#robotic"
nmcli connection modify robotic connection.autoconnect yes
nmcli connection modify robotic connection.autoconnect-priority 10

# WLAN "RoboCup"
nmcli connection add type wifi ifname wlan0 con-name "RoboCup" ssid "RoboCup"
nmcli connection modify RoboCup wifi-sec.key-mgmt wpa-psk wifi-sec.psk "#robotic"
nmcli connection modify RoboCup connection.autoconnect yes
nmcli connection modify RoboCup connection.autoconnect-priority 20

# WLAN "ROBU"
nmcli connection add type wifi ifname wlan0 con-name "ROBU" ssid "ROBU"
nmcli connection modify ROBU wifi-sec.key-mgmt wpa-psk wifi-sec.psk "#robotic"
nmcli connection modify ROBU connection.autoconnect yes
nmcli connection modify ROBU connection.autoconnect-priority 30

# WLAN "S-MISC"
nmcli connection add type wifi ifname wlan0 con-name "S-MISC" ssid "S-MISC"
nmcli connection modify S-MISC wifi-sec.key-mgmt wpa-psk wifi-sec.psk "sbCLEtuiDMD$#*8C"
nmcli connection modify S-MISC connection.autoconnect yes
nmcli connection modify S-MISC connection.autoconnect-priority 40
