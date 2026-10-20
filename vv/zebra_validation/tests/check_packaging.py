"""Exercise production packaging with an explicit fake executable, never a board image."""
from pathlib import Path
import argparse
import hashlib
import shutil
import subprocess
import sys
import tempfile
import zipfile

parser = argparse.ArgumentParser()
parser.add_argument("--bash", help="Optionally run the actual unzip/checksum deployment commands")
args = parser.parse_args()
source = Path(__file__).resolve().parents[1]

with tempfile.TemporaryDirectory(prefix="zebra-package-test-") as folder:
    root = Path(folder)
    for name in ["tools/package_zebra.py", "src/ZebraTest/Version.hpp", "run_zebra_test.sh", "config/zebra_test.ini"]:
        target = root / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / name, target)
    version = (root / "src/ZebraTest/Version.hpp").read_text(encoding="utf-8").split('Version="')[1].split('"')[0]
    fake = b"TEST FIXTURE ONLY, NOT EXECUTABLE\n" + version.encode() + b"\n"
    program = root / "vv_zebra_test"
    program.write_bytes(fake)
    command = [sys.executable, str(root / "tools/package_zebra.py")]
    subprocess.run(command, check=True, capture_output=True)
    package = root / "vv_zebra_test_board.zip"
    expected = {"vv_zebra_test", "run_zebra_test.sh", "config/zebra_test.ini", "ZEBRA_BUILD_INFO.txt", "ZEBRA_PROGRAM_SHA256SUMS"}
    with zipfile.ZipFile(package) as archive:
        assert set(archive.namelist()) == expected
        assert archive.testzip() is None
        assert archive.read("vv_zebra_test") == fake
        checksums = archive.read("ZEBRA_PROGRAM_SHA256SUMS").decode("ascii").splitlines()
        assert len(checksums) == 3
        for line in checksums:
            digest, name = line.split("  ")
            assert hashlib.sha256(archive.read(name)).hexdigest() == digest
        assert all("config/" not in line and "image_params" not in line for line in checksums)
    print("PASS package entries, CRC, current program bytes, checksums and tuning exclusions")
    previous = package.read_bytes()
    program.write_bytes(b"old binary without this version")
    result = subprocess.run(command, capture_output=True)
    assert result.returncode != 0 and b"version" in result.stderr
    assert package.read_bytes() == previous
    program.unlink()
    assert subprocess.run(command, capture_output=True).returncode != 0
    assert package.read_bytes() == previous
    print("PASS stale/missing program rejection; existing package left intact")
    # Rebuilding replaces the entire archive, so stale archive entries cannot survive.
    with zipfile.ZipFile(package, "a") as archive:
        archive.writestr("image_params.txt", "unwanted old template")
    program.write_bytes(fake + b"second build\n")
    subprocess.run(command, check=True, capture_output=True)
    with zipfile.ZipFile(package) as archive:
        assert set(archive.namelist()) == expected
        assert archive.read("vv_zebra_test") == program.read_bytes()
    print("PASS whole-archive replacement removes obsolete template entries")
    if args.bash:
        board = root / "board"
        (board / "config").mkdir(parents=True)
        image = b"steer_offset_deg=20\nexperiment_enable=0\n"
        tuning = b"far_frames=5\n# retained field tuning\n"
        (board / "image_params.txt").write_bytes(image)
        (board / "config/zebra_test.ini").write_bytes(tuning)
        script = root / "deploy.sh"
        script.write_bytes(b'''#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/board"
unzip -n ../vv_zebra_test_board.zip config/zebra_test.ini
unzip -o ../vv_zebra_test_board.zip -x config/zebra_test.ini
sha256sum -c ZEBRA_PROGRAM_SHA256SUMS
''')
        result = subprocess.run([args.bash, script.as_posix()], capture_output=True)
        if result.returncode:
            raise AssertionError((result.returncode, (result.stdout + result.stderr).decode("utf-8", errors="replace")))
        assert (board / "image_params.txt").read_bytes() == image
        assert (board / "config/zebra_test.ini").read_bytes() == tuning
        assert (board / "vv_zebra_test").read_bytes() == program.read_bytes()
        print("PASS actual unzip/checksum commands retain offset=20 and existing zebra tuning")
