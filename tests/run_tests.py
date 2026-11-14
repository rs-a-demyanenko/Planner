"""Native C++11 checks against pinned real Axis sources; Python stdlib only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / '.build'
PINS = {
    'Axis': ('rs-a-demyanenko/Axis', '55e855dbc42f560b08a481450cb2e694871f0b11'),
    'GyverStepper': ('GyverLibs/GyverStepper', 'e9d0a68833605e87a082a9367877434d6bb8f271'),
    'GyverMotor2': ('GyverLibs/GyverMotor2', '29972794183db1d92df00ef0e71a38dcac48545d'),
}


def unpack(archive, destination):
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive) as package:
        for member in package.namelist():
            if not (destination / member).resolve().is_relative_to(destination.resolve()):
                raise RuntimeError('Unsafe archive member')
        package.extractall(destination)


def dependencies():
    result = {}
    for name, (repo, commit) in PINS.items():
        folder = BUILD / 'deps' / (name + '-' + commit)
        if not folder.exists():
            archive = BUILD / (name + '.zip')
            print('Downloading', name, commit, flush=True)
            with urllib.request.urlopen(f'https://codeload.github.com/{repo}/zip/{commit}', timeout=60) as response:
                archive.write_bytes(response.read())
            unpack(archive, folder.parent)
            archive.unlink()
        result[name] = folder
    return result


def compiler(download):
    for name in ('g++', 'clang++'):
        if shutil.which(name):
            return [shutil.which(name)]
    portable = ROOT / '.tools' / 'zig-windows-x86_64-0.13.0' / 'zig.exe'
    zig = shutil.which('zig') or (str(portable) if portable.exists() else None)
    if not zig and download and sys.platform == 'win32':
        with urllib.request.urlopen('https://ziglang.org/download/index.json', timeout=60) as response:
            release = json.load(response)['0.13.0']['x86_64-windows']
        print('Downloading portable Zig 0.13.0', flush=True)
        with urllib.request.urlopen(release['tarball'], timeout=120) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != release['shasum']:
            raise RuntimeError('Compiler archive checksum mismatch')
        archive = BUILD / 'zig.zip'
        archive.write_bytes(data)
        unpack(archive, ROOT / '.tools')
        archive.unlink()
        zig = str(portable)
    if zig:
        return [zig, 'c++']
    raise RuntimeError('Install g++/clang++/Zig, or use --download-compiler on Windows')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--download-compiler', action='store_true')
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    BUILD.mkdir(exist_ok=True)
    deps = dependencies()
    cc = compiler(args.download_compiler)
    env = os.environ.copy()
    env['ZIG_GLOBAL_CACHE_DIR'] = str(BUILD / 'zig-global-cache')
    env['ZIG_LOCAL_CACHE_DIR'] = str(BUILD / 'zig-cache')
    common = cc + ['-std=c++11', '-Wall', '-Wextra', '-Werror', '-pedantic',
                   '-I' + str(ROOT / 'src'), '-I' + str(ROOT / 'tests' / 'support'),
                   '-isystem', str(deps['Axis'] / 'src'),
                   '-isystem', str(deps['GyverStepper'] / 'src'),
                   '-isystem', str(deps['GyverMotor2'])]
    if args.sanitize:
        common += ['-fsanitize=undefined', '-fno-sanitize-recover=all']
    output = BUILD / ('planner_tests.exe' if os.name == 'nt' else 'planner_tests')
    command = common + [str(ROOT / 'tests' / 'test_planner.cpp')]
    command += [str(deps['Axis'] / 'src' / file) for file in ('AxisCore.cpp', 'AxisProfile.cpp', 'AxisSync.cpp')]
    command += ['-o', str(output)]
    print('Compiling native integration tests', flush=True)
    subprocess.run(command, cwd=BUILD, env=env, check=True)
    subprocess.run([str(output)], cwd=ROOT, check=True)
    for sketch in sorted((ROOT / 'examples').glob('*/*.ino')):
        # Arduino sketch preprocessing is unnecessary: functions are declared
        # before use. This verifies C++ APIs against the actual driver headers.
        translation_unit = BUILD / (sketch.stem + '.cpp')
        shutil.copyfile(sketch, translation_unit)
        command = common + ['-c', str(translation_unit), '-o', str(BUILD / (sketch.stem + '.o'))]
        print('Checking example', sketch.parent.name, flush=True)
        subprocess.run(command, cwd=BUILD, env=env, check=True)
    print('PASS: native tests and example API checks', flush=True)


if __name__ == '__main__':
    main()
