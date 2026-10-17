# Validation scope

New steering range checks (20261017.4):

- 90 pure steering checks: decimal primary/correction commands beyond +/-90 accepted,
  exact original float arithmetic at five offsets including the measured 20, output without
  clamp/sign changes, electrical boundaries, overflow/nonfinite rejection before driver mutation,
  field-specific preflight failures including disabled correction values, retained stage directions.
- Race/manual runtime bridge fixtures reject an invalid later-stage pulse before any motion/logs,
  and deliver stage_2 command -120.5 unchanged with offset20 through the actual runtime bridge.
  Missing image parameters are refused rather than generated with a guessed offset.
- The fixed limit is removed only from parking config magnitude validation. All stored commands
  are preflighted before globalInit; every live output is checked using the same pulse helper.
  Time/PWM/direction/center constraints and static center-mode -5/0/+5 probes remain.
- Original pulse arithmetic is factored into a pure helper without changing its float operation
  order, offset application, PWM period or hardware call order. Physical range is not inferred.
- Three affected Linux translation units (main/runtime/steer) compile to Linux x86_64 objects
  with Zig and upstream OpenCV 4.10 headers. This is source compatibility, not a LoongArch link.
- Existing 57 state checks, 39 stationary checks and 17 PWM fixture checks remain part of verification.

Previous parking software checks (20261017.3):

- 39 stationary servo parser/output checks: motor writes stay zero, no arbitrary probe,
  zero before servo, failed motor-zero blocks steering, malformed/duplicate/nonfinite settings.
- 17 actual PwmController filesystem-fixture checks: existing channel reused without export,
  repeated initialization, delayed node creation, acknowledged zero writes, incomplete channels,
  invalid enable metadata, missing chip and failed writes preserving the last acknowledged cached value.
- Original Steer pulse arithmetic, Global initialization sequence, motor direction mapping,
  lane algorithm, image_params.txt and all numeric parking stage parameters are preserved.
- Packaging fixture validates both manifests, preservation of board tuning and refusal of x86 binaries.
- Updated build/launch scripts handle the actual VM download folder and board OpenCV paths.


- 57 pure C++ state/config/timing checks, including output acknowledgement, zero-reverse
  restart, duplicate triggers, decimal durations, stale camera, two bends, skipped stage,
  right-only/both-wheel reverse, malformed ranges, failed writes, manual standby, explicit
  authorization, fresh post-authorization line, mode roundtrip and manual acquisition timeout.
- Native runtime bridge tests in separate race/manual processes with fake Motor/Steer:
  initialization-time manual ownership, zero motors/no servo before authorization, no early log
  directory, ignored zebra events, rejected duplicate authorization, discarded pre-Enter frame,
  race handoff, blocked late image write,
  first forward output, camera-stage deadline, reverse output, failure zeros and six CSVs.
- Five affected Linux translation units (main/runtime/PWM/motor/steer) compiled after the
  center-mode change; eight integrated units had compiled in the prior version.
  They were compiled to x86_64 Linux objects using Zig and
  upstream OpenCV 4.10 public headers; this checks Linux source compatibility, not target linking.

Baseline limitations (not introduced or repaired in this parking task):

- The uploaded VV15 `roundabout_test` fails check 44 at line 68 under the native host compiler.
  The identical failure was reproduced against the unmodified archive source.
- Other original host fixtures fail at low_camera_ring_test:139, track_policy_test:56,
  entrance_probe_test:134. These tests and their Roundabout logic are unchanged from VV15
  (Git may normalize text line endings).
- Original unused-variable/sign warnings remain. No claim that all inherited tests pass.

No LoongArch OpenCV link or real-car test was possible on this Windows host. Run
`bash build.sh` with the actual LoongArch toolchain and target OpenCV; its package step
rejects binaries whose ELF machine type is not LoongArch (258). Route durations/PWM/angles
remain tuning parameters. Native encoder rps is diagnostic and not physical position proof.

The source archive supplied by the user and the previous parking project were not modified.
