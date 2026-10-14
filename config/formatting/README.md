# Gemeinsame Formatierungsregeln

- `.clang-format`: C/C++, vier Leerzeichen, 99 Zeichen, Allman-Klammern.
- `pyproject.toml`: Python/ROS 2, vier Leerzeichen, 99 Zeichen, einfache Strings;
  Ruff prüft zusätzlich Benennung, Imports und Type Hints.
- Versionen: Ruff **0.17.0**, clang-format auf dem PC **19.1.7**.

Das PC- und Pi-Setup verlinkt beide Dateien nach `~/work`. Neue Projekte darunter
verwenden die Regeln automatisch. Näher gelegene Projektkonfigurationen haben
Vorrang. Eine Änderung dieser Vorlagen gilt für alle Projekte, die sie erben.

Für eigenständige Projekte, auch außerhalb von `~/work`, im Projektordner ausführen:

```bash
python3 ~/work/.robu/robu_setup/setup_formatting.py --project .
```

Der Befehl kopiert beide Dateien ins Projekt; diese Kopien mit Git versionieren.
Vorhandene Dateien werden beibehalten. Eine bestehende `pyproject.toml` mit
Paketmetadaten kann um eigene `[tool.ruff]`-Regeln ergänzt werden.

VS Code formatiert per **Strg+Umschalt+I**, weiterhin manuell.
Die Dateinamen sind `.clang-format` und `pyproject.toml`.

Details: [Clang-Format-Konfiguration](https://clang.llvm.org/docs/ClangFormatStyleOptions.html#configuring-style-with-clang-format),
[Ruff-Konfigurationssuche](https://docs.astral.sh/ruff/configuration/#config-file-discovery).
