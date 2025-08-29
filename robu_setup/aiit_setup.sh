# HA

mkdir -p ~/work/java

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

#firefox --wait-for-browser https://htlkaindorfat-my.sharepoint.com/personal/li_htl-kaindorf_at/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fli%5Fhtl%2Dkaindorf%5Fat%2FDocuments%2FArbeit%2FHTL%2DKaindorf%2FLern%5Fund%5FLehrunterlagen%2FJava%2Fjava%2Ezip&parent=%2Fpersonal%2Fli%5Fhtl%2Dkaindorf%5Fat%2FDocuments%2FArbeit%2FHTL%2DKaindorf%2FLern%5Fund%5FLehrunterlagen%2FJava&ga=1
#echo "Firefox ist zu, Skript geht weiter..."


curl 'https://htlkaindorfat-my.sharepoint.com/personal/li_htl-kaindorf_at/_layouts/15/download.aspx?SourceUrl=%2Fpersonal%2Fli%5Fhtl%2Dkaindorf%5Fat%2FDocuments%2FArbeit%2FHTL%2DKaindorf%2FLern%5Fund%5FLehrunterlagen%2FJava%2Fjava%2Ezip' \
  -H 'User-Agent: Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:142.0) Gecko/20100101 Firefox/142.0' \
  -H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8' \
  -H 'Accept-Language: en-US,en;q=0.5' \
  -H 'Accept-Encoding: gzip, deflate, br, zstd' \
  -H 'Referer: https://htlkaindorfat-my.sharepoint.com/personal/li_htl-kaindorf_at/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fli%5Fhtl%2Dkaindorf%5Fat%2FDocuments%2FArbeit%2FHTL%2DKaindorf%2FLern%5Fund%5FLehrunterlagen%2FJava%2Fjava%2Ezip&parent=%2Fpersonal%2Fli%5Fhtl%2Dkaindorf%5Fat%2FDocuments%2FArbeit%2FHTL%2DKaindorf%2FLern%5Fund%5FLehrunterlagen%2FJava&ga=1' \
  -H 'Upgrade-Insecure-Requests: 1' \
  -H 'Sec-Fetch-Dest: iframe' \
  -H 'Sec-Fetch-Mode: navigate' \
  -H 'Sec-Fetch-Site: same-origin' \
  -H 'Connection: keep-alive' \
  -H 'Cookie: FedAuth=77u/PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0idXRmLTgiPz48U1A+VjE0LDBoLmZ8bWVtYmVyc2hpcHx1cm4lM2FzcG8lM2F0ZW5hbnRhbm9uI2I3ZWRkZTRjLTM3OGItNDczMS05MzJkLTdmNjZjMjRkYTc3MSwwIy5mfG1lbWJlcnNoaXB8dXJuJTNhc3BvJTNhdGVuYW50YW5vbiNiN2VkZGU0Yy0zNzhiLTQ3MzEtOTMyZC03ZjY2YzI0ZGE3NzEsMTM0MDA5NzQzNjUwMDAwMDAwLDAsMTM0MDEwNjA0NjUzOTU5NjMxLDAuMC4wLjAsMjU4LGI3ZWRkZTRjLTM3OGItNDczMS05MzJkLTdmNjZjMjRkYTc3MSwsLDJlOWNjMGExLTYwZTctZDAwMC1iOWY4LWVkOTIwYWViMWE1NCwyZTljYzBhMS02MGU3LWQwMDAtYjlmOC1lZDkyMGFlYjFhNTQsQW1XVWhwbFdGa3luY2lia0lwSHByQSwwLDAsMCwsLCwyNjUwNDY3NzQzOTk5OTk5OTk5LDAsLCwsLCwsMCwsMTg5MTIxLF9HaFdSeGxvMEtQV0lNdV9lOGxuNVpXTHZ0YywsbWlhRUtHNCtFbm1JL2kyaHc2MTVUeUtSdU1XQW10c2RlQzdTQ0xSVTZjdi80cENIN0hXdGIwMXVIK3Jic0lvT1o2MEsyeWxNR09oWTlEWUtRZCtwL2Q2cml1M1orWkhwTkhXZno4clFTQTBjVCtLY2VPOUl3M0lVY2VOSVpuSE43Z25hK3VmN1V5OEJqQTFKNmU0Y2tYZUo4SUpkSTcvUmdlS1F1M1hLdFI2d2ptWTVOdWEyNnhZL2J4eTFSWndUblJ4ZUNRQ3dxOGNVRjh1dlFPUDN4TEdra1dWQ1hDdzhjT2RqTEQ1VEFxY09PeU5EaUxDT3dBT0kzdWc1ejNpQ0EreVJGNFU3VlV1MHVwM214NXJUTjY0c3hNcHdBUTdRSm02Y0YvbzNZOEhJM1ZkNGFEVGhBRGVKTFFyczFLQ1ZjMmRlbEdpbVlDZDB4Z0k2N3ZFVytBPT08L1NQPg==; FeatureOverrides_experiments=[]; ai_session=Cmzz3tt00K2QVzEMyxyvXP|1756500467388|1756500467411; msal.cache.encryption=%7B%22id%22%3A%220198f796-1f77-7be9-84ad-5e12c8af5030%22%2C%22key%22%3A%22TR8EZRsvmnxihM3hxp0Ub72coxhadd4hGGECTi1wCME%22%7D; MSFPC=GUID=b004eee94f0942bf8e6ac594fe70bfe3&HASH=b004&LV=202508&V=4&LU=1756500470083' -o ~/work/java.zip


#Doku extrahieren
unzip -o -q ~/work/java.zip -d ~/work/

#Heruntergeladene Datei nach der Installation wieder löschen
rm ~/work/java.zip
