#!/usr/bin/env bash
set -euo pipefail

# Quelle (Repo) und Ziel (VS Code Snippet-Ordner)
SRC="${1:-$HOME/work/.robu/config/snippets}"
DST="${2:-$HOME/.config/Code/User/snippets}"

echo "SRC: $SRC"
echo "DST: $DST"

mkdir -p "$DST"

find "$SRC" -type f -name "*.code-snippets" | while read -r file; do
  base="$(basename "$file")"
  ln -sf "$file" "$DST/$base"
  echo "Linked: $base"
done

echo "Done. Bitte VS Code neu laden (Developer: Reload Window)."
