"""Compile/link with installed Arduino cores; never upload firmware."""
import argparse
from pathlib import Path
import shutil
import subprocess

from run_tests import ROOT, BUILD, dependencies


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cli', default=shutil.which('arduino-cli') or 'arduino-cli')
    parser.add_argument('--config')
    parser.add_argument('--board', default='arduino:avr:mega')
    parser.add_argument('--example', action='append', help='Default: all examples')
    args = parser.parse_args()
    BUILD.mkdir(exist_ok=True)
    deps = dependencies()
    examples = args.example or [p.name for p in sorted((ROOT / 'examples').iterdir()) if p.is_dir()]
    for name in examples:
        if not (ROOT / 'examples' / name / (name + '.ino')).is_file():
            parser.error('Unknown example: ' + name)
        suffix = args.board.replace(':', '-') + '-' + name
        cli = str(Path(args.cli).resolve()) if Path(args.cli).is_file() else args.cli
        command = [cli, 'compile', '--fqbn', args.board,
                   '--build-path', str(BUILD / suffix), '--library', str(ROOT), '--warnings', 'all']
        if args.config:
            command += ['--config-file', str(Path(args.config).resolve())]
        for folder in deps.values():
            command += ['--library', str(folder)]
        command += [str(ROOT / 'examples' / name)]
        print('Building', args.board, name, flush=True)
        result = subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, encoding='utf-8', errors='replace')
        (BUILD / (suffix + '.log')).write_text(result.stdout, encoding='utf-8')
        for line in result.stdout.splitlines():
            if line.startswith(('Sketch uses ', 'Global variables use ')):
                print(line, flush=True)
        if result.returncode:
            print(result.stdout, flush=True)
            result.check_returncode()
    print('PASS: selected Arduino examples compiled and linked', flush=True)


if __name__ == '__main__':
    main()
