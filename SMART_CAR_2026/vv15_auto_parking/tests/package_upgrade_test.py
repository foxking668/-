#!/usr/bin/env python3
"""Exercise packaging in a temporary fixture; never execute its synthetic ELF header."""
from pathlib import Path
import hashlib
import shutil
import subprocess
import sys
import tempfile
import zipfile

source = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="vv15-package-") as temporary:
    root = Path(temporary)
    (root / "tools").mkdir()
    (root / "config").mkdir()
    shutil.copyfile(source / "tools/package_board.py", root / "tools/package_board.py")
    required = ["run_auto_parking.sh", "start_parking.sh", "run_servo_center.sh", "image_params.txt",
                "config/parking_tuning.vv.ini", "AUTO_PARKING.md", "PARKING_PARAMETERS.md", "SERVO_CENTER.md", "ORIGIN.md"]
    for name in required:
        shutil.copyfile(source / name, root / name)
    header = bytearray(20)
    header[:4] = b"\x7fELF"
    header[18:20] = (258).to_bytes(2, "little")
    (root / "vv_auto_parking").write_bytes(header)
    subprocess.run([sys.executable, str(root / "tools/package_board.py")], check=True, capture_output=True)
    destination = root / "board"
    (destination / "config").mkdir(parents=True)
    tuned = {"image_params.txt": b"steer_offset_deg=82.5\n",
             "config/parking_tuning.vv.ini": b"# custom stage times and angles\nsteer_command=-60\n"}
    for name, data in tuned.items():
        (destination / name).write_bytes(data)
    def records(data):
        return dict(line.split("  ", 1)[::-1] for line in data.decode().splitlines())
    with zipfile.ZipFile(root / "vv15_auto_parking_board.zip") as package:
        assert package.testzip() is None
        full = records(package.read("AUTO_PARKING_SHA256SUMS"))
        program = records(package.read("AUTO_PARKING_PROGRAM_SHA256SUMS"))
        assert set(full) - set(program) == set(tuned)
        for name, digest in full.items():
            assert hashlib.sha256(package.read(name)).hexdigest() == digest
        # Equivalent to documented unzip -o archive -x image_params.txt config/parking_tuning.vv.ini.
        for name in package.namelist():
            if name in tuned:
                continue
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(package.read(name))
    assert all((destination / name).read_bytes() == value for name, value in tuned.items())
    assert all(hashlib.sha256((destination / name).read_bytes()).hexdigest() == digest
               for name, digest in program.items())
    header[18:20] = (62).to_bytes(2, "little")
    (root / "vv_auto_parking").write_bytes(header)
    refused = subprocess.run([sys.executable, str(root / "tools/package_board.py")], capture_output=True)
    assert refused.returncode != 0 and b"LoongArch ELF" in refused.stderr
print("PASS board manifests / parameter-preserving upgrade / wrong-architecture rejection")
