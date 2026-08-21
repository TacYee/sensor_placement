# V4 build notes

This version reads the state from Crazyflie log variables instead of using estimator/stabilizer APIs:

- `stateEstimate.x`
- `stateEstimate.y`
- `stateEstimate.z`
- `stateEstimate.yaw`

Main behavior:

1. Receive waypoints through app channel command `0xA1`.
2. Start with `traj.start = 1`.
3. Track trajectory.
4. Detect collision by low actual XY progress while command speed is high.
5. Stop briefly.
6. Move servo clockwise.
7. Back up.
8. Land.

Manual trigger:

```bash
traj.forceCollision = 1
```

Build:

```bash
cd ~/crazyflie-bl/crazyflie-firmware/examples/demos/app_servo_test_whole
rm -rf build cache
make -j$(nproc)
make cload
```

If your compiler still reports `-Wno-stringop-overread`, remove that flag from the firmware-level Makefile/Kbuild file where it appears.
