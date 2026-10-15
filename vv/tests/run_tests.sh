#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
CONES_ONLY=0
if [ "${1:-}" = "--cones-only" ]; then CONES_ONLY=1; shift; fi
TEST_BIN="$(mktemp /tmp/vv-roundabout-test.XXXXXX)"
TEST_VIDEO_DIR="$(mktemp -d /tmp/vv-video-test.XXXXXX)"
TEST_LIVE_DIR="$(mktemp -d /tmp/vv-live-test.XXXXXX)"
TEST_START_DIR="$(mktemp -d /tmp/vv-start-test.XXXXXX)"
TEST_GATE_DIR="$(mktemp -d /tmp/vv-gate-test.XXXXXX)"
trap 'rm -f "$TEST_BIN"; rm -rf "$TEST_VIDEO_DIR" "$TEST_LIVE_DIR" "$TEST_START_DIR" "$TEST_GATE_DIR"' EXIT
c++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -UNDEBUG -Isrc \
    tests/cone_sequence_test.cpp -o "$TEST_BIN"
"$TEST_BIN" "$PWD/cone_avoidance.ini"
if [ "$CONES_ONLY" -eq 0 ]; then
c++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -D_GLIBCXX_ASSERTIONS -Isrc \
    tests/roundabout_test.cpp src/NewTrack/Roundabout.cpp -o "$TEST_BIN"
"$TEST_BIN"
c++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -D_GLIBCXX_ASSERTIONS -Isrc \
    tests/low_camera_ring_test.cpp src/NewTrack/Roundabout.cpp -o "$TEST_BIN"
"$TEST_BIN"
c++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -D_GLIBCXX_ASSERTIONS -Isrc \
    tests/track_policy_test.cpp src/NewTrack/Roundabout.cpp -o "$TEST_BIN"
"$TEST_BIN"
# Optional integration with the original contour algorithm and host OpenCV.
c++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -D_GLIBCXX_ASSERTIONS -Isrc \
    tests/entrance_probe_test.cpp src/NewTrack/Roundabout.cpp src/NewTrack/EightNeighbor.cpp -o "$TEST_BIN"
"$TEST_BIN"
fi
if command -v pkg-config >/dev/null && pkg-config --exists opencv4; then
    c++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -UNDEBUG -Isrc \
        tests/binarization_test.cpp src/NewTrack/Binarization.cpp \
        $(pkg-config --cflags --libs opencv4) -o "$TEST_BIN"
    "$TEST_BIN"
    c++ -std=c++17 -O1 -UNDEBUG -Isrc tests/cone_boundary_test.cpp src/NewTrack/ConeBoundary.cpp \
        $(pkg-config --cflags --libs opencv4) -o "$TEST_BIN"
    "$TEST_BIN"
    c++ -std=c++17 -O1 -UNDEBUG -pthread -Isrc tests/diagnostic_video_test.cpp \
        src/NewTrack/DiagnosticView.cpp src/Experiment/VideoRecorder.cpp \
        $(pkg-config --cflags --libs opencv4) -o "$TEST_BIN"
    "$TEST_BIN" "$TEST_VIDEO_DIR"
    c++ -std=c++17 -O1 -UNDEBUG -pthread -Isrc tests/recording_start_test.cpp \
        src/Experiment/ProcessedRecording.cpp src/Experiment/VideoRecorder.cpp \
        $(pkg-config --cflags --libs opencv4) -o "$TEST_START_DIR/start_test"
    "$TEST_START_DIR/start_test" "$TEST_START_DIR"
    c++ -std=c++17 -O1 -UNDEBUG -ffunction-sections -Isrc -Isrc/Experiment/Baselines \
        tests/live_pipeline_test.cpp src/NewTrack/NewTrack.cpp src/NewTrack/Roundabout.cpp \
        src/NewTrack/TrackGates.cpp src/NewTrack/EightNeighbor.cpp src/NewTrack/ConeBoundary.cpp \
        src/NewTrack/DiagnosticView.cpp src/NewTrack/Binarization.cpp src/ImageProcess/FindLine/FindLine.cpp \
        src/ImageProcess/FindStartPos/FindStartPos.cpp src/ImageProcess/FixLine/FixLine.cpp \
        src/ImageProcess/HandleImage/HandleImage.cpp src/Experiment/Baselines/baselines.cpp \
        src/Other/Global/Global.cpp \
        -Wl,--gc-sections $(pkg-config --cflags --libs opencv4) -o "$TEST_BIN"
    "$TEST_BIN" "$TEST_LIVE_DIR"
    c++ -std=c++17 -O1 -UNDEBUG -ffunction-sections -Isrc -Isrc/Experiment/Baselines \
        tests/pipeline_gate_test.cpp src/NewTrack/NewTrack.cpp src/NewTrack/Roundabout.cpp \
        src/NewTrack/TrackGates.cpp src/NewTrack/EightNeighbor.cpp src/NewTrack/ConeBoundary.cpp \
        src/NewTrack/DiagnosticView.cpp src/NewTrack/Binarization.cpp src/ImageProcess/FindLine/FindLine.cpp \
        src/ImageProcess/FindStartPos/FindStartPos.cpp src/ImageProcess/FixLine/FixLine.cpp \
        src/ImageProcess/HandleImage/HandleImage.cpp src/Experiment/Baselines/baselines.cpp \
        src/Other/Global/Global.cpp \
        -Wl,--gc-sections $(pkg-config --cflags --libs opencv4) -o "$TEST_BIN"
    "$TEST_BIN" "$TEST_GATE_DIR"
    if [ "$CONES_ONLY" -eq 0 ]; then
    c++ -std=c++17 -O1 -Isrc tests/track_gates_test.cpp src/NewTrack/TrackGates.cpp \
        $(pkg-config --cflags --libs opencv4) -o "$TEST_BIN"
    "$TEST_BIN"
    c++ -std=c++17 -O1 -Isrc tests/roundabout_vision_test.cpp src/NewTrack/Roundabout.cpp \
        src/ImageProcess/FindLine/FindLine.cpp src/ImageProcess/FindStartPos/FindStartPos.cpp \
        src/ImageProcess/HandleImage/HandleImage.cpp $(pkg-config --cflags --libs opencv4) -o "$TEST_BIN"
    "$TEST_BIN" "$@"
    c++ -std=c++17 -O1 -Isrc tests/eight_neighbor_vision_test.cpp src/NewTrack/Roundabout.cpp \
        src/NewTrack/EightNeighbor.cpp src/ImageProcess/HandleImage/HandleImage.cpp \
        $(pkg-config --cflags --libs opencv4) -o "$TEST_BIN"
    "$TEST_BIN" "$@"
    fi
elif [ "$CONES_ONLY" -eq 1 ]; then
    echo 'Cone integration checks require pkg-config opencv4; no visual tests were run.' >&2
    exit 1
fi
