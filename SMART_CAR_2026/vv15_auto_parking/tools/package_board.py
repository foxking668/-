#!/usr/bin/env python3
"""Package the actual cross-compiled executable and its runtime configuration."""
from pathlib import Path
import hashlib
import zipfile

root = Path(__file__).resolve().parents[1]
binary = root / "vv_auto_parking"
if not binary.is_file():
    raise SystemExit("Build vv_auto_parking first")
header = binary.read_bytes()[:20]
if header[:4] != b"\x7fELF" or len(header) < 20 or int.from_bytes(header[18:20], "little") != 258:
    raise SystemExit("Expected a LoongArch ELF executable; refusing to package a host binary")
paths = [binary, root / "run_auto_parking.sh", root / "image_params.txt",
         root / "config/parking_tuning.vv.ini", root / "AUTO_PARKING.md", root / "PARKING_PARAMETERS.md", root / "ORIGIN.md"]
paths += sorted(root.glob("ring_*.png"))
checks = "".join(f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(root).as_posix()}\n" for p in paths)
archive = root / "vv15_auto_parking_board.zip"
with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as out:
    for path in paths:
        out.write(path, path.relative_to(root).as_posix())
    out.writestr("AUTO_PARKING_SHA256SUMS", checks)
print(f"BOARD_PACKAGE {archive} ({archive.stat().st_size} bytes)")
