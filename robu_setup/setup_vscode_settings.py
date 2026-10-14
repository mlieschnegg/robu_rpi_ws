#!/usr/bin/env python3
"""Apply Python-first VS Code user settings. Run as the desktop user, not root."""

import argparse
import json
import os
import shutil
import tempfile
from pathlib import Path

import json5


def configure(
    settings_path: Path | str,
    os_release_path: Path | str = '/etc/os-release',
    remote: bool = False,
    venv_dir: Path | str | None = None,
) -> None:
    settings_path = Path(settings_path)
    original = settings_path.read_text(encoding='utf-8-sig') if settings_path.exists() else None
    settings = json5.loads(original) if original and original.strip() else {}
    if not isinstance(settings, dict):
        raise ValueError('VS Code settings must be a JSON object; file left unchanged')

    venv_dir = Path(venv_dir or Path.home() / 'work/.venvs/robu').expanduser().absolute()
    templates_dir = Path(__file__).resolve().parents[1] / 'config/formatting'
    manual_formatting = {
        'editor.formatOnSave': False,
        'editor.formatOnPaste': False,
        'editor.formatOnType': False,
        'editor.insertSpaces': True,
        'editor.tabSize': 4,
        'editor.detectIndentation': False,
    }
    settings.update(manual_formatting)
    settings.update(
        {
            'files.eol': '\n',
            'python.defaultInterpreterPath': str(venv_dir / 'bin/python'),
            'ruff.path': [str(venv_dir / 'bin/ruff')],
            'ruff.nativeServer': 'on',
            'ruff.configurationPreference': 'filesystemFirst',
            'ruff.configuration': str(templates_dir / 'pyproject.toml'),
            'C_Cpp.clang_format_style': 'file',
            'C_Cpp.clang_format_path': '' if remote else str(venv_dir / 'bin/clang-format'),
        }
    )
    for language, formatter in (
        ('[python]', 'charliermarsh.ruff'),
        ('[c]', 'ms-vscode.cpptools'),
        ('[cpp]', 'ms-vscode.cpptools'),
    ):
        preferences = settings.setdefault(language, {})
        if not isinstance(preferences, dict):
            raise ValueError(f'{language} settings must be an object; file left unchanged')
        preferences.update(manual_formatting)
        preferences['editor.defaultFormatter'] = formatter
    for preferences in (settings, settings['[python]']):
        actions = preferences.setdefault('editor.codeActionsOnSave', {})
        if not isinstance(actions, dict):
            raise ValueError('editor.codeActionsOnSave must be an object; file left unchanged')
        actions.update(
            {
                'source.fixAll': 'never',
                'source.fixAll.ruff': 'never',
                'source.organizeImports': 'never',
                'source.organizeImports.ruff': 'never',
            }
        )

    # Disable both semantic C++ analysis and the Tag Parser for Python lessons.
    # Keep the extension installed for PlatformIO/debugging and opt-in C++ work.
    settings.update(
        {
            'C_Cpp.intelliSenseEngine': 'disabled',
            'C_Cpp.intelliSenseCacheSize': 0,
            'C_Cpp.default.browse.limitSymbolsToIncludedHeaders': True,
        }
    )
    if remote:
        # On the 2 GB Pi even Pylance light mode can consume too much RAM.
        # This is written ONLY to the server's Remote User settings.
        settings['python.languageServer'] = 'None'
    else:
        extensions = settings.setdefault('remote.SSH.defaultExtensions', [])
        if not isinstance(extensions, list):
            raise ValueError('remote.SSH.defaultExtensions must be an array; file left unchanged')
        for extension in ('ms-python.python', 'charliermarsh.ruff', 'ms-vscode.cpptools'):
            if extension not in extensions:
                extensions.append(extension)
        settings.update(
            {
                'python.analysis.languageServerMode': 'default',
                'python.analysis.indexing': True,
                'python.analysis.diagnosticMode': 'openFilesOnly',
            }
        )

    # Match ros_setup.sh without requiring ROS to be installed already.
    # Preserve additional paths, including paths to custom interface packages.
    os_release = Path(os_release_path)
    release = os_release.read_text() if os_release.exists() else ''
    ros_paths = []
    if 'VERSION_ID="24.04"' in release:
        ros_paths = [
            '/opt/ros/jazzy/lib/python3.12/site-packages',
            '/opt/ros/jazzy/local/lib/python3.12/dist-packages',
        ]
    elif 'VERSION_ID="22.04"' in release:
        ros_paths = [
            '/opt/ros/humble/lib/python3.10/site-packages',
            '/opt/ros/humble/local/lib/python3.10/dist-packages',
        ]
    if ros_paths and not remote:
        paths = settings.setdefault('python.analysis.extraPaths', [])
        if not isinstance(paths, list):
            raise ValueError('python.analysis.extraPaths must be an array; file left unchanged')
        for path in ros_paths:
            if path not in paths:
                paths.append(path)

    updated = json.dumps(settings, indent=4, ensure_ascii=False) + '\n'
    if original == updated:
        print(f'[ROBU] VS Code settings already configured: {settings_path}')
        return

    settings_path.parent.mkdir(parents=True, exist_ok=True)
    # One fixed backup keeps the original comments without growing on each run.
    backup = settings_path.with_name(settings_path.name + '.pre-robu.bak')
    if original is not None and not backup.exists():
        shutil.copy2(settings_path, backup)

    # Atomic replacement avoids a partial settings file if writing is interrupted.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', newline='\n', dir=settings_path.parent, delete=False
        ) as stream:
            temporary = Path(stream.name)
            stream.write(updated)
        if settings_path.exists():
            shutil.copymode(settings_path, temporary)
        temporary.replace(settings_path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    print(f'[ROBU] Configured VS Code user settings: {settings_path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--remote',
        action='store_true',
        help='Configure Raspberry Pi Remote-SSH settings; disable Python language server',
    )
    parser.add_argument(
        '--server-dir', type=Path, help='Custom VS Code server directory (requires --remote)'
    )
    parser.add_argument(
        '--venv-dir',
        type=Path,
        default=Path.home() / 'work/.venvs/robu',
        help='Shared Python development environment',
    )
    args = parser.parse_args()
    if args.server_dir and not args.remote:
        parser.error('--server-dir requires --remote')
    if os.geteuid() == 0:
        raise SystemExit('Run this helper as the desktop user, without sudo.')
    if args.remote:
        server_dir = args.server_dir or Path(
            os.environ.get('VSCODE_AGENT_FOLDER') or Path.home() / '.vscode-server'
        )
        configure(
            server_dir.expanduser() / 'data' / 'Machine' / 'settings.json',
            remote=True,
            venv_dir=args.venv_dir,
        )
        print('[ROBU] Reconnect Remote-SSH or run Developer: Reload Window to apply.')
    else:
        config_dir = Path(os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config')
        configure(config_dir / 'Code' / 'User' / 'settings.json', venv_dir=args.venv_dir)
