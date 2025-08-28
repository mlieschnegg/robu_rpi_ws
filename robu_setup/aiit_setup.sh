# HA

mkdir -p ~/work/java

sudo apt install -y openjdk-21-jdk
sudo snap install netbeans --classic

#Java 21 JavaDoc \url{https://www.oracle.com/java/technologies/javase-jdk21-doc-downloads.html} https://download.oracle.com/otn_software/java/jdk/21.0.8+12/c927acdc99414b95a38389e236910a05/jdk-21.0.8_doc-all.zip
#JavaBuch (\url{https://www.javabuch.de/download.html} ==> html.zip, examples.zip) https://www.javabuch.de/download/html.zip, https://www.javabuch.de/download/examples.zip
#JavaInsel (\url{https://www.heise.de/download/product/java-ist-auch-eine-insel-49226})

#sudo nano /etc/gdm3/custom.conf, find the line #WaylandEnable=false
sudo sed -Ei \
 's/^[[:space:]]*#[[:space:]]*WaylandEnable=false/WaylandEnable=false/; s/^[[:space:]]*WaylandEnable=true/WaylandEnable=false/' \
 /etc/gdm3/custom.conf

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
  -H 'Cookie: FedAuth=77u/PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0idXRmLTgiPz48U1A+VjE0LDBoLmZ8bWVtYmVyc2hpcHx1cm4lM2FzcG8lM2F0ZW5hbnRhbm9uI2I3ZWRkZTRjLTM3OGItNDczMS05MzJkLTdmNjZjMjRkYTc3MSwwIy5mfG1lbWJlcnNoaXB8dXJuJTNhc3BvJTNhdGVuYW50YW5vbiNiN2VkZGU0Yy0zNzhiLTQ3MzEtOTMyZC03ZjY2YzI0ZGE3NzEsMTM0MDA4NTA3MDIwMDAwMDAwLDAsMTM0MDA5MzY4MDI0NDA3MTE5LDAuMC4wLjAsMjU4LGI3ZWRkZTRjLTM3OGItNDczMS05MzJkLTdmNjZjMjRkYTc3MSwsLDNmMjZjMGExLWEwYzQtZDAwMC1iOWY4LWUzYmZkODM5OWFkNCwzZjI2YzBhMS1hMGM0LWQwMDAtYjlmOC1lM2JmZDgzOTlhZDQsY2hvWHRIR2hha3U5STRjUTVVbTZmdywwLDAsMCwsLCwyNjUwNDY3NzQzOTk5OTk5OTk5LDAsLCwsLCwsMCwsMTg5MTIxLF9HaFdSeGxvMEtQV0lNdV9lOGxuNVpXTHZ0YywsamlSMDd6dEVIUW1tREwxRW9KSXczQVNzZWluaHV3YWxiMFg3VHlYZHpyLy80c0hicmtEQmk0ZTFtdHpjRWt6L3lHMWYrMTVuUUtQcnpjVDBvSjBJMHpKY2lndGd3Q2lkZ1pWV2VWSGNUU1gyeENtM3hRQ1llNWRxWDB1QklvNzl3MTROUlplSTk2K1FFTFUxYmNYdG14bk9mMkRRTFk2UW9rbzQwTGYwQUNPeUVEckJrdkZNdnk5MkZ5M3lMOWRwWllYc0lPUjhZSEpCVWdCTzBXazBmK2t4NktzNDJ4eUYrWk5HUXFIS2NaRVlNUytxcW5JUGFPZElEMnV3cGFmT3lBTy9XVkc4TmcwRkZjU21XVWplblRNdDNCZ2d2WmNWZVVod0dCUEo5RFprT3M3T3RqejhXdFdCUmZrTlA5Sm9jQXVJRW1kRXdXMFJDMG1tVkh4aGZ3PT08L1NQPg==; FeatureOverrides_experiments=[]; ai_session=pZcXqAWj6qhQenciLkYyz9|1756376804378|1756377201191; msal.cache.encryption=%7B%22id%22%3A%220198f037-2d22-7414-8cb8-a76e27eb5ab4%22%2C%22key%22%3A%225NkubjgTfstwpMH9xPcVbwx_ZqodbdSf0KhagMpuEwQ%22%7D; MSFPC=GUID=d409e6f950704dd79fcc05e1f4f41172&HASH=d409&LV=202508&V=4&LU=1756376813704; SPA_RT=' -o ~/work/java.zip


unzip -o -q ~/work/java.zip -d ~/work/

rm ~/work/java.zip
