"""Check delivery provenance, exact parameters, and extraction boundaries; no hardware access."""
from pathlib import Path
import hashlib
import json
import re

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_PATCHES = {
    "src/NewTrack/NewTrack.cpp", "src/NewTrack/NewTrack.hpp",
    "src/HandleProcess/ImageHandle/ImageHandle.cpp",
    "src/HandleProcess/MotorHandle/MotorHandle.cpp",
}


def check():
    manifest = json.loads((ROOT / "ORIGIN_SHA256.json").read_text(encoding="utf-8"))
    changed = set()
    for relative, digest in manifest["files"].items():
        actual = ROOT / "vv" / relative
        assert actual.is_file(), f"missing upstream dependency: {relative}"
        if hashlib.sha256(actual.read_bytes()).hexdigest() != digest:
            changed.add(relative)
    assert changed == EXPECTED_PATCHES, changed
    # Motor, encoder, PWM, steering formulas, PID, camera, zebra gates and the
    # complete S2/S4/S5 controller remain byte-identical to the uploaded ZIP.
    assert (ROOT / "image_params.txt").read_bytes() == (ROOT / "vv/image_params.txt").read_bytes()
    entries = []
    for line in (ROOT / "image_params.txt").read_text(encoding="utf-8-sig").splitlines():
        line = line.split("#", 1)[0].strip()
        if "=" in line:
            entries.append(tuple(piece.strip() for piece in line.split("=", 1)))
    parameters = dict(entries)
    assert len(parameters) == len(entries), "duplicate parameter keys"
    expected = {
        "new_track_zebra_finish_stop_delay_ms": "4000", "motor_aim_speed": "7",
        "center_pratio": "0.48", "error_gain": "3.5", "steer_offset_deg": "95",
        "new_track_post_stop_center_deg": "102", "new_track_post_stop_settle_ms": "200",
        "new_track_zebra_stop_ms": "3200", "new_track_zebra_reverse_ms": "800",
        "new_track_zebra_reverse_pwm": "12000", "new_track_zebra_reverse_left_enable": "0",
        "new_track_post_stop_left_motor_enable": "0", "new_track_post_stop_right_motor_enable": "1",
    }
    for stage, offset, total, hold in [("s2", "-107", "2450", "2000"), ("s4", "108", "3500", "2000"), ("s5", "0", "1700", "0")]:
        prefix = "new_track_post_stop_" + stage
        expected.update({prefix+"_enable": "1", prefix+"_steer_deg": offset,
                         prefix+"_pwm": "12700", prefix+"_ms": total, prefix+"_hold_ms": hold})
    assert all(parameters[k] == v for k, v in expected.items())
    pipeline = (ROOT / "vv/src/NewTrack/NewTrack.cpp").read_text(encoding="utf-8")
    # Confirm there is one request builder shared by detector and manual stage selection.
    assert len(re.findall(r"static Parking::Request makeParkingRequest\(", pipeline)) == 1
    assert pipeline.count("makeParkingRequest(c, zebra.stopEndMs)") == 1
    assert pipeline.count("makeParkingRequest(cfg(),0)") == 1
    assert "lapCounter.store(1);" in pipeline
    assert "if(ParkingTest::runtime().enabled()) isolateStandaloneTasks(c);" in pipeline
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert 'list(REMOVE_ITEM VV_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/vv/src/main.cpp")' in cmake
    assert not (ROOT / "vv/vv").exists(), "old competition binary must not masquerade as standalone"
    assert not (ROOT / "vv_parking_test").exists() or (ROOT / "vv_parking_test").is_file()
    print(f"PASS extraction: {len(manifest['files'])} upstream files, 4 isolated hook files, {len(entries)} exact parameters")


if __name__ == "__main__":
    check()
