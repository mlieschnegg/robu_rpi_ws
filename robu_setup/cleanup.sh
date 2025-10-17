#!/bin/bash

gh auth logout -h github.com
sudo rm -f ~/.gitconfig
sudo rm -f -r ~/.config/gh/


rm -rf ~/snap/firefox/
rm -rf ~/.cache/vscode-cpptools/*
rm -rf ~/.cache/pip/*
rm -rf ~/Downloads/*
rm -rf ~/.python_history
rm -rf ~/.bash_history
rm -rf ~/.local/share/Trash/{files,info}/*

rm -rf ~/ROBU/
rm -rf ~/KOP/
rm -rf ~/AIIT/
rm -rf ~/robotic/
rm  -rf ~/java/plfs/
rm  -rf ~/java/uebungen/prj/*
rm  -rf ~/java/uebungen/src/*
rm  -rf ~/java/.git/
rm  -rf ~/java/README.md

snap list --all | awk '/disabled/{print $1, $3}' | while read snapname revision; do sudo snap remove --purge "$snapname" --revision="$revision"; done

