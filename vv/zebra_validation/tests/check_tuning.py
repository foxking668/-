"""Reject dangerous or ambiguous tuning before the program touches hardware."""
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
exe = Path(sys.argv[1]).resolve()
baseline = (root / "config/zebra_test.ini").read_text(encoding="utf-8")
cases = {
    "short_hold": (baseline.replace("hold_ms=3000", "hold_ms=2999"), "CONFIG_HOLD"),
    "duplicate": (baseline + "\nhold_ms=3000\n", "CONFIG_DUPLICATE"),
    "unknown": (baseline + "\nreverse_left_enable=1\n", "CONFIG_UNKNOWN"),
    "nan": (baseline.replace("stationary_rps=0.4", "stationary_rps=nan"), "CONFIG_NUMBER"),
    "reverse_limit": (baseline.replace("reverse_max_ms=800", "reverse_max_ms=0"), "CONFIG_REVERSE"),
    "invalid_roi": (baseline.replace("near_top=0.30", "near_top=0.99"), "CONFIG_NEAR_ROI"),
    "even_window": (baseline.replace("illumination_band_rows=9", "illumination_band_rows=8"), "CONFIG_ILLUMINATION_BAND"),
    "startup_timeout": (baseline.replace("startup_timeout_ms=5000", "startup_timeout_ms=0"), "CONFIG_STARTUP_TIMEOUT"),
}
with tempfile.TemporaryDirectory(prefix="zebra-tuning-") as temp:
    for name, (contents, expected) in cases.items():
        path = Path(temp) / f"{name}.ini"
        path.write_text(contents, encoding="utf-8")
        result = subprocess.run([str(exe), str(path)], capture_output=True)
        output = (result.stdout + result.stderr).decode("utf-8", errors="replace")
        if result.returncode == 0 or expected not in output:
            raise AssertionError((name, result.returncode, output))
        print("PASS", name, expected)
