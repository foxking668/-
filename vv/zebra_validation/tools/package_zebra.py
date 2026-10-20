"""Build the board ZIP from the just-built executable; never ship image_params.txt."""
from pathlib import Path
import hashlib
import os
import re
import tempfile
import zipfile

root = Path(__file__).resolve().parents[1]
version = re.search(r'Version="([^"]+)"', (root / "src/ZebraTest/Version.hpp").read_text(encoding="utf-8")).group(1)
program = root / "vv_zebra_test"
if not program.is_file():
    raise SystemExit("Missing vv_zebra_test: run bash build_zebra.sh first")
if version.encode() not in program.read_bytes():
    raise SystemExit("Executable version does not match source; rebuild before packaging")
(root / "ZEBRA_BUILD_INFO.txt").write_bytes((version + "\n").encode("utf-8"))
files = ["vv_zebra_test", "run_zebra_test.sh", "ZEBRA_BUILD_INFO.txt"]
checksums = "".join(hashlib.sha256((root / name).read_bytes()).hexdigest() + "  " + name + "\n" for name in files)
(root / "ZEBRA_PROGRAM_SHA256SUMS").write_bytes(checksums.encode("ascii"))
files += ["config/zebra_test.ini", "ZEBRA_PROGRAM_SHA256SUMS"]
fd, temporary = tempfile.mkstemp(prefix=".zebra-board-", suffix=".tmp", dir=root)
os.close(fd)
try:
    with zipfile.ZipFile(temporary, "w", zipfile.ZIP_DEFLATED) as archive:
        for name in files:
            archive.write(root / name, name)
    os.replace(temporary, root / "vv_zebra_test_board.zip")
finally:
    if os.path.exists(temporary):
        os.unlink(temporary)
print(version)
print("Built vv_zebra_test_board.zip (no image_params.txt; preserve the car's tuning)")
