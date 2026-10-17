# Source baseline

Imported from the user-supplied `F:\共享文件夹\vv(15).zip`.

Archive SHA-256: `9684fabdb1a2449ed5440af9da52e186b779af20f68b98c998cf7fe42d7093de`.

The uploaded archive, the existing `vv_parking` project, and other cars are untouched.
The baseline hardware definitions, pin mappings, encoder conversion, motor initialization
order and servo formula remain VV15. New code adds the second-zebra parking handoff,
per-stage raw PWM, motor-thread timers, output acknowledgements and CSV recording.
PWM file writes now check stream errors; motor/steer setters return write success.
Only the parking stage uses these acknowledgements to start its timer.

Version 20261017.4 removes only the parking command's fixed +/-90 range checks.
The original Steer pulse arithmetic (including float operation order) is factored into
SteeringPulse.hpp for identical preflight and actual output checks. Hardware initialization,
direction mapping and numeric example parameters are preserved. Keep the board's measured
steer_offset_deg=20 by excluding image_params.txt when updating its program package.

The IMU debug command inherited from VV15 is not used for parking. No IMU feedback,
ultrasonic terminal positioning, or visual reverse-stage control is introduced.

Build products, source-archive backup files and the nested NewTrack archive are excluded.
