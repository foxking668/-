"""Package a built LoongArch recorder and matching launcher; never execute it."""
import argparse
import hashlib
from pathlib import Path
import re
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
PROGRAM_NAME = 'manual_capture_auto_probe_20261007'
SCRIPT_NAME = 'run_auto_reverse_probe.sh'
MANIFEST_NAME = 'AUTO_PROBE_SHA256SUMS'


def source_version():
    source = (ROOT / 'tools/manual_capture.cpp').read_text(encoding='utf-8')
    found = re.search(r'recorderVersion="(\d{4}-\d{2}-\d{2}\.\d+)"', source)
    if not found:
        raise ValueError('Cannot identify recorder source version')
    return found.group(1)


def package_program(program_path, output_path):
    version = source_version()
    program = Path(program_path).read_bytes()
    if len(program) < 64 or program[:4] != b'\x7fELF':
        raise ValueError('Recorder is not an ELF program; zero-filled or wrong input')
    if program[4:6] != b'\x02\x01' or int.from_bytes(program[18:20], 'little') != 258:
        raise ValueError('Recorder must be a little-endian 64-bit LoongArch ELF program')
    if version.encode('ascii') not in program:
        raise ValueError('Recorder does not contain current source version ' + version + '; rebuild first')
    script = (ROOT / 'deploy' / SCRIPT_NAME).read_bytes()
    if not script.startswith(b'#!/bin/sh\n') or b'\r' in script:
        raise ValueError('Launcher must be LF shell text')
    if ('version=' + version).encode('ascii') not in script:
        raise ValueError('Launcher and recorder source versions disagree')
    payloads = {PROGRAM_NAME: program, SCRIPT_NAME: script}
    payloads[MANIFEST_NAME] = ''.join(
        hashlib.sha256(data).hexdigest() + '  ' + name + '\n'
        for name, data in payloads.items()).encode('ascii')
    output = Path(output_path).resolve()
    if output.suffix.lower() != '.zip' or output == Path(program_path).resolve():
        raise ValueError('Output must be a ZIP path separate from the program')
    output.parent.mkdir(parents=True, exist_ok=True)
    # Build and verify a separate archive before replacing the destination.
    with tempfile.NamedTemporaryFile(dir=output.parent, prefix='auto-probe-', suffix='.zip', delete=False) as scratch:
        temporary = Path(scratch.name)
    try:
        with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
            for name, data in payloads.items():
                info = zipfile.ZipInfo(name)
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = (0o100644 if name == MANIFEST_NAME else 0o100755) << 16
                archive.writestr(info, data)
        with zipfile.ZipFile(temporary) as archive:
            if archive.testzip() is not None or set(archive.namelist()) != set(payloads):
                raise ValueError('Package CRC or file list verification failed')
            for name, data in payloads.items():
                if archive.read(name) != data:
                    raise ValueError('Package content mismatch: ' + name)
        temporary.replace(output)
    finally:
        if temporary.exists():
            temporary.unlink()
    return output, payloads[MANIFEST_NAME].decode('ascii')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--program', required=True, help='Built LoongArch manual_capture executable')
    version = source_version()
    date, revision = version.split('.')
    default = ROOT / 'deploy' / ('auto_reverse_probe_' + date.replace('-', '') + '_' + revision + '_verified.zip')
    parser.add_argument('--output', default=str(default))
    args = parser.parse_args()
    output, manifest = package_program(args.program, args.output)
    print('VERIFIED_PACKAGE ' + str(output))
    print('zip_bytes=' + str(output.stat().st_size))
    print('zip_sha256=' + hashlib.sha256(output.read_bytes()).hexdigest())
    print(manifest, end='')


if __name__ == '__main__':
    main()
