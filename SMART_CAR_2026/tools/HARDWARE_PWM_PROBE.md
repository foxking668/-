# PWM read-return diagnostic

The board reported `Short device transfer /dev/zf_device_pwm_motor_1: expected 24, got 0`.
The supplied factory `zf_driver_file.cpp` treats every nonnegative syscall return
as success; the vehicle's `LinuxDeviceIo` requires the requested byte count.
Neither the factory wrapper nor the error log proves whether a zero-return read
actually populates the buffer. The kernel driver implementation is not yet verified.

`hardware_pwm_probe` opens the three configured PWM devices read-only, twice each
with complementary initial byte patterns. It prints raw return counts, changed-byte
counts, the full buffer, and six candidate fields. It does not read encoders or write
motor, servo, direction or GPIO commands. It uses the existing vehicle lock and
hardware configuration. Plausible fields are evidence only; the tool does not
declare a zero-return read successful or change the vehicle I/O policy. Exit code 2
indicates nonstandard/failed reads. Individual device failures do not hide later devices.

In the existing x86 Linux VM, using the configured cross-build directory:

```bash
cd /mnt/hgfs/share/SMART_CAR_2026
cmake -S . -B /home/gy/builds/SMART_CAR_2026-loongarch
cmake --build /home/gy/builds/SMART_CAR_2026-loongarch --target hardware_pwm_probe --parallel 2
cp /home/gy/builds/SMART_CAR_2026-loongarch/hardware_pwm_probe /mnt/hgfs/share/SMART_CAR_2026/deploy/
```

Upload only `hardware_pwm_probe` to `/home/root/gy/deploy/`, then run on the board:

```bash
cd /home/root/gy/deploy
chmod +x hardware_pwm_probe
./hardware_pwm_probe --hardware-config config/hardware.ini
```

This target does not link OpenCV. Do not run it on the x86 VM: it is a LoongArch binary.
If returned counts are zero but buffers change into coherent metadata in both samples,
inspect the running kernel driver's copy/return convention before designing a narrowly
scoped compatibility fix. If buffers remain initial patterns, zero-return acceptance
would disguise missing data. No PWM write or encoder behavior can be inferred from
this read-only probe.
