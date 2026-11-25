"""Check the mechanical BARR-C:2018 subset used by this firmware.

This is not a complete standards audit. See firmware/BARR_C_2018.md for
scope, compatibility exceptions, manual review and verification evidence.
Requires Python 3.10+; --format additionally requires clang-format 19+.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OPAQUE = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', re.S)


def owned_files() -> list[Path]:
    """Exclude libraries, generated files, test doubles and local secrets."""
    directories = [ROOT / 'firmware/common', ROOT / 'firmware/esp32_s3_vision/main']
    for board in ('robo_pico', 'sensor_pico'):
        directories.extend(ROOT / 'firmware' / board / name
                           for name in ('include', 'src', 'camera_monitor', 'motor_bench'))
    return sorted({p for directory in directories for p in directory.glob('*')
                   if p.suffix in ('.c', '.h')
                   and p.name not in ('config.h', 'wifi_secrets.h')})


def formatter_path(explicit: str | None) -> str | None:
    if explicit:
        return explicit
    found = os.environ.get('CLANG_FORMAT') or shutil.which('clang-format')
    if found:
        return found
    # Visual Studio installs LLVM without necessarily adding it to PATH.
    visual_studio = Path(os.environ.get('ProgramFiles', 'C:/Program Files')) / 'Microsoft Visual Studio'
    return next(
        (str(p) for p in visual_studio.glob('*/*/VC/Tools/Llvm/x64/bin/clang-format.exe')), None)


def check_file(path: Path) -> list[str]:
    failures = []
    relative = path.relative_to(ROOT).as_posix()
    data = path.read_bytes()
    try:
        source = data.decode('utf-8')
    except UnicodeDecodeError:
        return [f'{relative}: source must be UTF-8']
    if any(byte < 32 and byte not in (10, 12) for byte in data) or b'\x7f' in data:
        failures.append(f'{relative}: tabs/CR/control characters are prohibited')
    if not data.endswith(b'\n'):
        failures.append(f'{relative}: missing final newline')
    if not source.rstrip().endswith('/* End of file. */'):
        failures.append(f'{relative}: missing end-of-file comment')
    for number, line in enumerate(source.splitlines(), 1):
        if len(line) > 80:
            failures.append(f'{relative}:{number}: exceeds 80 columns ({len(line)})')
    code = OPAQUE.sub(lambda m: '\n' * m[0].count('\n'), source)
    for match in re.finditer(r'\b(auto|register|short|long)\b', code):
        number = code.count('\n', 0, match.start()) + 1
        failures.append(f'{relative}:{number}: prohibited keyword {match[0]}')
    if path.suffix == '.h' and not re.search(r'^#ifndef\s+[A-Z][A-Z_0-9]+\s*\n#define\s+', code, re.M):
        failures.append(f'{relative}: missing conventional include guard')
    if path.name not in ('FreeRTOSConfig.h', 'wifi_secrets.example.h') and not re.fullmatch(r'[a-z][a-z_0-9]*\.[ch]', path.name):
        failures.append(f'{relative}: module name must be lowercase')
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--format', action='store_true', help='also check clang-format output')
    parser.add_argument('--fix-format', action='store_true', help='apply the shared clang-format layout')
    parser.add_argument('--formatter', help='explicit clang-format executable')
    args = parser.parse_args()
    files = owned_files()
    if args.format or args.fix_format:
        formatter = formatter_path(args.formatter)
        if not formatter:
            parser.error('clang-format not found; set CLANG_FORMAT or pass --formatter')
        result = subprocess.run([formatter, '-i' if args.fix_format else '--dry-run',
                                 *([] if args.fix_format else ['--Werror']),
                                 *(str(p) for p in files)], check=False)
        if result.returncode:
            return result.returncode
    failures = [failure for path in files for failure in check_file(path)]
    if failures:
        print('\n'.join(failures), file=sys.stderr)
        return 1
    print(f'Mechanical BARR-C checks passed for {len(files)} project-owned C/header files.')
    print('Manual rules and documented compatibility exceptions still require review.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
