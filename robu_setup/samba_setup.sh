sudo apt install -y samba

FILE='/etc/samba/smb.conf'
# Wenn [work] noch nicht existiert → Block anhängen
if ! grep -Eq '^\s*\[work\]\s*$' "$FILE"; then
  printf '\n%s\n' "[work]
comment = Arbeitsverzeichnis der SuS
path = /home/robu/work
read only = no
browsable = yes" | sudo tee -a "$FILE" >/dev/null
  echo "Abschnitt [work] angehängt."
else
  echo "Abschnitt [work] ist bereits vorhanden – nichts zu tun."
fi

if sudo pdbedit -L -v | grep -Eq "^[[:space:]]*Unix username:[[:space:]]*${SUDO_USER:-$USER}([[:space:]]|$)"; then
    echo "Es wurde bereits ein Passwort für die Samba-Freigabe gesetzt"
else
    #sudo useradd -d /home/$USER -s /sbin/nologin $USER
    sudo smbpasswd -a $USER
fi

sudo service smbd restart
sudo ufw allow samba
