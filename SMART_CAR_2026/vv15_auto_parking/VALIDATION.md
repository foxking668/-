# Validation scope

New parking software checks:

- 40 pure C++ state/config/timing checks, including output acknowledgement, zero-reverse
  restart, duplicate triggers, decimal durations, stale camera, two bends, skipped stage,
  right-only/both-wheel reverse, malformed ranges, and failed writes.
- Native runtime bridge test with fake Motor/Steer: handoff, blocked late image write,
  first forward output, camera-stage deadline, reverse output, failure zeros and six CSVs.
- Eight modified Linux translation units compiled to x86_64 Linux objects using Zig and
  upstream OpenCV 4.10 public headers; this checks Linux source compatibility, not target linking.

Baseline limitations (not introduced or repaired in this parking task):

- The uploaded VV15 `roundabout_test` fails check 44 at line 68 under the native host compiler.
  The identical failure was reproduced against the unmodified archive source.
- Other original host fixtures fail at low_camera_ring_test:139, track_policy_test:56,
  entrance_probe_test:134. These tests and their Roundabout source are byte-identical to VV15.
- Original unused-variable/sign warnings remain. No claim that all inherited tests pass.

No LoongArch OpenCV link or real-car test was possible on this Windows host. Run
`bash build.sh` with the actual LoongArch toolchain and target OpenCV; its package step
rejects binaries whose ELF machine type is not LoongArch (258). Route durations/PWM/angles
remain tuning parameters. Native encoder rps is diagnostic and not physical position proof.

The source archive supplied by the user and the previous parking project were not modified.
