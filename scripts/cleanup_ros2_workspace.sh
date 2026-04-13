#!/bin/bash

# Setze das Home-Verzeichnis
home_dir="$HOME"

# Lösche alle browse.vc.db-Dateien
echo "Suche nach browse.vc.db Dateien und lösche sie..."
find "$home_dir" -type f -name "browse.vc.db" -exec rm -f {} \;

# Aufräumen von ROS2 Workspaces (build, log, install Ordner löschen), mit Ausnahme der spezifischen Pfade
echo "Suche nach ROS2 Workspaces und lösche build, log, install Ordner, außer bei ~/home/robu/work/.robu und ~/home/robu/work/turtlebot3_ws..."

# Verzeichnisse, die nicht aufgeräumt werden sollen
exclude_dirs=("$home_dir/work/.robu" "$home_dir/work/turtlebot3_ws" "$home_dir/work/microros_ws")

# Durchlaufe alle Verzeichnisse im Home-Verzeichnis und lösche die Ordner build, log, install, außer in den ausgeschlossenen Pfaden
find "$home_dir" -type d \( -name "build" -o -name "log" -o -name "install" \) | while read dir; do
    # Prüfe, ob das Verzeichnis in den ausgeschlossenen Pfaden ist
    exclude=false
    for exclude_dir in "${exclude_dirs[@]}"; do
        if [[ "$dir" == "$exclude_dir"* ]]; then
            exclude=true
            break
        fi
    done
    
    # Wenn das Verzeichnis nicht ausgeschlossen ist, lösche es
    if [ "$exclude" = false ]; then
        echo "Lösche: $dir"
        rm -rf "$dir"
    else
        echo "Überspringe: $dir"
    fi
done

echo "Aufräumen abgeschlossen!"