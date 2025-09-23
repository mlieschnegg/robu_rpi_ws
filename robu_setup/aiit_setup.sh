# HA

mkdir -p ~/work/java

DOWNLOAD_ONEDRIVE_URL="https://htlkaindorfat-my.sharepoint.com/:u:/g/personal/li_htl-kaindorf_at/EV9nPpBUOiJEitT-13-Rt5EBO7ykCPCJRlbKptmt8W2EUQ?download=1"
DOWNLOAD_FILE=~/work/java.zip

sudo apt install -y openjdk-21-jdk
sudo snap install netbeans --classic

#Wayland deaktivieren!
sudo sed -Ei \
 's/^[[:space:]]*#[[:space:]]*WaylandEnable=false/WaylandEnable=false/; s/^[[:space:]]*WaylandEnable=true/WaylandEnable=false/' \
 /etc/gdm3/custom.conf

#Doku für JAVA Unterricht herunterladen und das Verzeichnis work/java kopieren
#Java 21 JavaDoc \url{https://www.oracle.com/java/technologies/javase-jdk21-doc-downloads.html} https://download.oracle.com/otn_software/java/jdk/21.0.8+12/c927acdc99414b95a38389e236910a05/jdk-21.0.8_doc-all.zip
#JavaBuch (\url{https://www.javabuch.de/download.html} ==> html.zip, examples.zip) https://www.javabuch.de/download/html.zip, https://www.javabuch.de/download/examples.zip
#JavaInsel (\url{https://www.heise.de/download/product/java-ist-auch-eine-insel-49226})

curl -fL --cookie-jar /tmp/od_c.jar --cookie /tmp/od_c.jar \
     -A "Mozilla/5.0" \
     -o $DOWNLOAD_FILE \
     "$DOWNLOAD_ONEDRIVE_URL"


#Doku extrahieren
unzip -o -q "$DOWNLOAD_FILE" -d ~/work/

#Heruntergeladene Datei nach der Installation wieder löschen
rm "$DOWNLOAD_FILE"

mkdir -p ~/java/uebungen/prj
mkdir -p ~/java/uebungen/src


#TODO: copy code templates (org-netbeans-modules-editor-settings-CustomCodeTemplates.xml)
# to: /home/robu/snap/netbeans/current/config/Editors/text/x-java/CodeTemplates/
