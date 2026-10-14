#!/usr/bin/env python3
"""Link shared formatter defaults into ~/work or copy them into a project."""

import argparse
import os
import sys
import tempfile
from pathlib import Path

CONFIG_FILES: tuple[str, str] = ('.clang-format', 'pyproject.toml')
DEFAULT_TEMPLATES_DIR: Path = Path(__file__).resolve().parents[1] / 'config' / 'formatting'


def _copy_config(source: Path, target: Path) -> None:
    # A hard link commits the complete temporary file without replacing existing files.
    with tempfile.NamedTemporaryFile(dir=target.parent) as temporary:
        temporary.write(source.read_bytes())
        temporary.flush()
        os.chmod(temporary.name, 0o644)
        os.link(temporary.name, target)


def install_formatting_configs(
    directory: Path,
    templates_dir: Path = DEFAULT_TEMPLATES_DIR,
    copy_files: bool = False,
) -> bool:
    directory = directory.expanduser().resolve()
    templates_dir = templates_dir.expanduser().resolve()
    successful = True
    for name in CONFIG_FILES:
        source = templates_dir / name
        target = directory / name
        try:
            if not source.is_file():
                raise FileNotFoundError(f'Missing formatter template: {source}')
            if target.exists() or target.is_symlink():
                print(f'[ROBU] Kept existing formatter config: {target}')
                continue
            directory.mkdir(parents=True, exist_ok=True)
            if copy_files:
                _copy_config(source, target)
                print(f'[ROBU] Copied formatter config: {target}')
            else:
                target.symlink_to(os.path.relpath(source, directory))
                print(f'[ROBU] Linked formatter config: {target} -> {source}')
        except OSError as error:
            print(f'[ROBU] Cannot install {target}: {error}', file=sys.stderr)
            successful = False
    return successful


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    locations = parser.add_mutually_exclusive_group()
    locations.add_argument('--work-dir', type=Path, default=Path.home() / 'work')
    locations.add_argument('--project', type=Path, help='Copy portable configs into this project')
    parser.add_argument('--templates-dir', type=Path, default=DEFAULT_TEMPLATES_DIR)
    args = parser.parse_args()
    if os.geteuid() == 0:
        parser.error('Run this helper as the development user, without sudo.')
    successful = install_formatting_configs(
        args.project if args.project is not None else args.work_dir,
        args.templates_dir,
        copy_files=args.project is not None,
    )
    raise SystemExit(0 if successful else 1)


if __name__ == '__main__':
    main()
