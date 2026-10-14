#!/usr/bin/env python3
"""Expose PlatformIO commands while keeping its isolated Python out of PATH."""

import argparse
import os
import re
import shlex
import shutil
import sys
import tempfile
from pathlib import Path

DEFAULT_REPO_DIR = Path(__file__).resolve().parents[1]
BLOCK_START = '# >>> ROBU tool paths >>>'
BLOCK_END = '# <<< ROBU tool paths <<<'
LEGACY_LINES = {
    'export PATH="$HOME/.platformio/penv/bin:$PATH"',
    'export PATH="$PATH:$HOME/.platformio/penv/bin"',
    'export PATH="$HOME/.local/bin:$PATH"',
}


def install_shell_config(bashrc: Path, repo_dir: Path) -> None:
    fragment = repo_dir.expanduser().resolve() / 'config' / 'shell' / 'tool_paths.sh'
    if not fragment.is_file():
        raise FileNotFoundError(f'Missing shell config: {fragment}')
    bashrc = bashrc.expanduser().absolute()
    if bashrc.is_symlink():
        raise ValueError(f'Cannot replace a symlink: {bashrc}')
    original = bashrc.read_text() if bashrc.exists() else ''
    if original.count(BLOCK_START) != original.count(BLOCK_END):
        raise ValueError(f'Incomplete ROBU block in {bashrc}; left unchanged')
    pattern = rf'^{re.escape(BLOCK_START)}\n.*?^{re.escape(BLOCK_END)}(?:\n|$)'
    updated, blocks = re.subn(pattern, '', original, flags=re.MULTILINE | re.DOTALL)
    if blocks != original.count(BLOCK_START):
        raise ValueError(f'Invalid ROBU block in {bashrc}; left unchanged')
    # Migrate the block used for the initial repair, as well as older installer lines.
    updated = updated.replace(fragment.read_text(), '')
    updated = ''.join(
        line for line in updated.splitlines(keepends=True) if line.strip() not in LEGACY_LINES
    )
    try:
        relative = fragment.relative_to(Path.home())
        source = '"$HOME"/' + shlex.quote(str(relative))
    except ValueError:
        source = shlex.quote(str(fragment))
    block = f'{BLOCK_START}\nif [ -f {source} ]; then\n    source {source}\nfi\n{BLOCK_END}\n'
    updated = updated.rstrip('\n') + '\n\n' + block
    if updated == original:
        print(f'[ROBU] Shell config already current: {bashrc}')
        return
    bashrc.parent.mkdir(parents=True, exist_ok=True)
    backup = bashrc.with_name(bashrc.name + '.pre-robu-tool-paths.bak')
    if bashrc.exists() and not backup.exists():
        shutil.copy2(bashrc, backup)
    mode = bashrc.stat().st_mode & 0o777 if bashrc.exists() else 0o644
    with tempfile.NamedTemporaryFile(mode='w', dir=bashrc.parent, delete=False) as temporary:
        temporary_path = Path(temporary.name)
        try:
            temporary.write(updated)
            temporary.flush()
            os.chmod(temporary_path, mode)
            os.replace(temporary_path, bashrc)
        finally:
            temporary_path.unlink(missing_ok=True)
    print(f'[ROBU] Updated shell config: {bashrc}')


def link_platformio_commands(platformio_bin: Path, launcher_dir: Path, required: bool) -> bool:
    platformio_bin = platformio_bin.expanduser().absolute()
    launcher_dir = launcher_dir.expanduser().absolute()
    successful = True
    for name in ('pio', 'platformio', 'piodebuggdb'):
        source = platformio_bin / name
        target = launcher_dir / name
        try:
            if not source.is_file() or not os.access(source, os.X_OK):
                if required and name != 'piodebuggdb':
                    raise FileNotFoundError(f'Missing PlatformIO launcher: {source}')
                continue
            launcher_dir.mkdir(parents=True, exist_ok=True)
            if target.is_symlink() and target.resolve() == source.resolve():
                continue
            if target.exists() and not target.is_symlink():
                print(f'[ROBU] Kept existing user command: {target}')
                continue
            with tempfile.TemporaryDirectory(dir=launcher_dir) as directory:
                temporary = Path(directory) / name
                temporary.symlink_to(source)
                os.replace(temporary, target)
            print(f'[ROBU] Linked command: {target} -> {source}')
        except OSError as error:
            print(f'[ROBU] Cannot install {target}: {error}', file=sys.stderr)
            successful = False
    return successful


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-dir', type=Path, default=DEFAULT_REPO_DIR)
    parser.add_argument('--bashrc', type=Path, default=Path.home() / '.bashrc')
    parser.add_argument(
        '--platformio-bin', type=Path, default=Path.home() / '.platformio/penv/bin'
    )
    parser.add_argument('--launcher-dir', type=Path, default=Path.home() / '.local/bin')
    parser.add_argument(
        '--link-platformio', action='store_true', help='Require installed Core commands'
    )
    args = parser.parse_args()
    if os.geteuid() == 0:
        parser.error('Run this helper as the development user, without sudo.')
    successful = True
    try:
        install_shell_config(args.bashrc, args.repo_dir)
    except (OSError, ValueError) as error:
        print(f'[ROBU] Cannot update shell config: {error}', file=sys.stderr)
        successful = False
    linked = link_platformio_commands(args.platformio_bin, args.launcher_dir, args.link_platformio)
    raise SystemExit(0 if successful and linked else 1)


if __name__ == '__main__':
    main()
