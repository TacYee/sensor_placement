# Sensor Placement Demo for Crazyflie

This repository contains the Crazyflie firmware and Python code used for the sensor-placement and landing-frame installation demo.

During the mission, the Crazyflie receives a pre-generated XY trajectory over CRTP, follows the trajectory using an onboard finite-state machine (FSM), detects contact, actuates a servo, moves backward, and lands. The Python script also records flight data for later analysis.

> **Safety warning:** This project controls a flying robot and an external actuator. Test the firmware and servo with the propellers removed before attempting a real flight. Use a clear test area and make sure an emergency-stop procedure is available.

## 1. Repository structure

The custom application is located in:

```text
examples/demos/app_servo_test_whole_fsm/
```

The main files are:

| File | Purpose |
|---|---|
| `src/app_main.c` | Main app loop and bridge between the Crazyflie state estimate and mission FSM |
| `src/mission_fsm.c` | Mission states for hovering, trajectory following, contact, servo actuation, backward flight, and landing |
| `src/servo_test.c` | Servo-control logic |
| `installationdrone.py` | Sends the trajectory, starts the mission, and records flight logs |
| `logcfg.json` | Logging configuration |
| `data/installation_drone_logs/` | Recorded flight logs |
| `app-config` | Firmware configuration for this out-of-tree app |
| `Makefile` | Builds the app together with the Crazyflie firmware |

The onboard mission follows this sequence:

```text
IDLE -> HOVERING -> FOLLOWING -> FLY_FORWARD -> CONTACT
     -> SERVO_CW -> FLY_BACKWARD -> LANDING -> FINISHED
```

## 2. Hardware requirements

- Crazyflie 2.1 Brushless
- Crazyradio or Crazyradio PA
- Servo and installation mechanism connected as expected by the firmware
- Charged Crazyflie battery
- Computer capable of running the Crazyflie development tools
- Clear and controlled flight area

The current servo implementation expects the servo signal on **TX2 / PA2**. Check the wiring and servo power requirements before applying power.

## 3. Set up the Crazyflie development environment

Before using this repository, follow the official Bitcraze building and flashing guide:

https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/

Complete the official setup before continuing. In particular, make sure that:

- the Crazyflie ARM toolchain is installed;
- Crazyradio is recognized by your computer;
- the Crazyflie client (`cfclient`) is installed;
- `cfclient` can connect to your Crazyflie;
- you are able to build and flash standard Crazyflie firmware.

Windows users should follow the WSL instructions in the same official guide.

Once `cfclient` can connect to the Crazyflie and the normal flashing workflow works, continue below.

## 4. Clone this repository

Open a Linux or WSL terminal. Choose a directory whose path contains no spaces, then run:

```bash
git clone --recursive https://github.com/TacYee/sensor_placement.git
cd sensor_placement
```

Do not clone the standard Bitcraze `crazyflie-firmware` repository instead. This repository already contains the Crazyflie firmware together with the custom sensor-placement app.

If the repository was cloned without `--recursive`, initialize the submodules manually:

```bash
git submodule update --init --recursive
```

Check that the Crazyflie Python library is available in the same Python environment used by `cfclient`:

```bash
python3 -c "import cflib; print('cflib is ready')"
```

If `cflib` is missing, install it in the current Python environment:

```bash
python3 -m pip install cflib
```

## 5. Build the sensor-placement firmware

From the repository root, enter the custom app directory:

```bash
cd examples/demos/app_servo_test_whole_fsm
```

Configure the firmware for the Crazyflie 2.1 Brushless:

```bash
make cf21bl_defconfig
```

Build the firmware:

```bash
make -j$(nproc)
```

This is an out-of-tree app build. It automatically builds the Crazyflie firmware together with the sensor-placement application. The standard firmware does not need to be built separately first.

After a successful build, inspect the generated firmware files:

```bash
find build -maxdepth 2 -type f \( -name "*.bin" -o -name "*.elf" -o -name "*.hex" \)
```

For the Crazyflie 2.1 Brushless, the generated binary is normally:

```text
build/cf21bl.bin
```

## 6. Flash the firmware

### Option A: wireless flashing with Crazyradio

1. Turn the Crazyflie off.
2. Hold the power button for approximately three seconds.
3. Release the button when both blue LEDs start blinking.
4. From the app directory, run:

```bash
make cload
```

Only one Crazyflie within radio range should be in bootloader mode.

To target a specific Crazyflie URI, keep the Crazyflie powered on and run:

```bash
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E8" make cload
```

Replace the URI with the address of your Crazyflie if it is different.

The generated binary can also be flashed directly:

```bash
cfloader flash build/cf21bl.bin stm32-fw \
  -w radio://0/80/2M/E7E7E7E7E8
```

### Option B: flashing with a debug adapter

If an ST-Link-compatible debug adapter and OpenOCD are installed, run:

```bash
make flash
```

## 7. Check the mission files

All commands below should be run from:

```text
sensor_placement/examples/demos/app_servo_test_whole_fsm/
```

Display the options supported by the mission script:

```bash
python3 installationdrone.py --help
```

Check that the default trajectory and logging configuration exist:

```bash
ls -lh next_drone_full_trajectory_landing_frame_20260522_132342.csv
ls -lh logcfg.json
```

If a different trajectory is used, replace the CSV filename in the commands below.

## 8. Perform a dry run

Before connecting to or flying the Crazyflie, verify that the trajectory can be loaded:

```bash
python3 installationdrone.py \
  --traj_csv next_drone_full_trajectory_landing_frame_20260522_132342.csv \
  --dry_run
```

The dry run previews the trajectory without starting a real flight.

If it fails, check that:

- the CSV file exists;
- the filename and path are correct;
- `cflib` is installed in the active Python environment;
- all other Python packages reported by the error are installed.

## 9. Pre-flight checklist

Complete every item before starting a real mission:

- [ ] The firmware builds without errors.
- [ ] The custom firmware has been flashed successfully.
- [ ] `cfclient` can connect to the Crazyflie.
- [ ] The battery is sufficiently charged.
- [ ] The servo wiring has been checked.
- [ ] The servo direction has been tested without propellers.
- [ ] The installation mechanism moves without jamming.
- [ ] The correct trajectory CSV has been selected.
- [ ] The drone is placed at the expected starting location.
- [ ] The flight area is clear.
- [ ] The operator is ready to stop the mission if required.

## 10. Run the mission

From the app directory, run:

```bash
python3 installationdrone.py \
  --uri radio://0/80/2M/E7E7E7E7E8 \
  --traj_csv next_drone_full_trajectory_landing_frame_20260522_132342.csv \
  --fileroot data/installation_drone_logs \
  --logconfig logcfg.json \
  --filename install_test1 \
  --enable_mission_log \
  --enable_traj_log \
  --enable_velocity_log \
  --velocity_config_name mission_nd \
  --init_servo_ccw
```

Replace the URI and trajectory filename when necessary.

The script will:

1. connect to the Crazyflie;
2. reset the mission and trajectory flags;
3. move the servo to its counter-clockwise starting position;
4. reset the estimator;
5. arm the Crazyflie;
6. send the trajectory points over CRTP;
7. start the onboard mission;
8. record the requested logs.

## 11. Stop the mission

To stop the script during flight, press:

```text
Ctrl+C
```

The script will attempt to request an onboard landing and disarm before exiting. Always keep the test area clear and maintain an independent emergency-stop procedure in case communication is lost.

## 12. Optional contact-detection test

The state-machine transition can be tested by requesting a simulated collision event:

```bash
python3 installationdrone.py \
  --uri radio://0/80/2M/E7E7E7E7E8 \
  --traj_csv next_drone_full_trajectory_landing_frame_20260522_132342.csv \
  --force-collision-after 8 \
  --init_servo_ccw
```

Use this only in a controlled test. Run the following command to check the exact meaning of the option:

```bash
python3 installationdrone.py --help
```

## 13. How the mission communication works

The Python script sends the XY trajectory to the Crazyflie as CRTP packets. A final packet containing:

```text
NaN, NaN
```

marks the end of the trajectory. The mission begins when the application sets:

```text
traj.start = 1
```

The servo is controlled through the `servotest.cmd` parameter:

| Value | Command |
|---:|---|
| `1` | Move counter-clockwise |
| `2` | Move clockwise |

These commands are handled by `src/servo_test.c` and triggered by the onboard mission FSM.

## 14. Flight logs

The recorded logs are written to:

```text
examples/demos/app_servo_test_whole_fsm/data/installation_drone_logs/
```

The default logging configuration is:

```text
examples/demos/app_servo_test_whole_fsm/logcfg.json
```

After a flight, list the newest log files with:

```bash
ls -lht data/installation_drone_logs | head
```

The Jupyter notebooks in the app directory can be used to inspect the recorded flight data.

## 15. Troubleshooting

### `arm-none-eabi-gcc: command not found`

The Crazyflie ARM toolchain is missing. Return to the official Bitcraze building and flashing guide and complete the toolchain setup.

### `No module named cflib`

Install `cflib` in the Python environment being used to run the mission:

```bash
python3 -m pip install cflib
```

### Crazyradio is not detected

Check whether the radio appears as a USB device:

```bash
lsusb
```

Then follow the official Bitcraze USB-permission instructions:

https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/

After updating the USB rules, unplug and reconnect the Crazyradio.

### The firmware builds for the wrong board

Clean and configure the app again:

```bash
make clean
make cf21bl_defconfig
make -j$(nproc)
```

### `make cload` cannot find the Crazyflie

Check that:

- the Crazyradio is connected;
- the USB permissions are configured;
- the Crazyflie is in bootloader mode;
- only one nearby Crazyflie is in bootloader mode;
- the specified URI is correct.

### The trajectory does not start

Check that:

- the custom firmware has been flashed;
- the selected trajectory CSV exists;
- all trajectory points were transmitted;
- the final `NaN, NaN` packet was sent;
- `traj.start` was set to `1`;
- the terminal shows the expected CRTP receiver messages.

### The servo does not move

Check:

- the servo power supply and ground connection;
- the signal connection to TX2 / PA2;
- the configured servo limits and pulse widths;
- whether `--init_servo_ccw` was included;
- whether the custom firmware was flashed successfully.

Test the servo without propellers before attempting another flight.

## 16. Quick-start summary

After completing the official Crazyflie setup and confirming that `cfclient` can connect to the drone:

```bash
git clone --recursive https://github.com/TacYee/sensor_placement.git
cd sensor_placement/examples/demos/app_servo_test_whole_fsm

make cf21bl_defconfig
make -j$(nproc)
make cload

python3 installationdrone.py \
  --traj_csv next_drone_full_trajectory_landing_frame_20260522_132342.csv \
  --dry_run
```

After the dry run and pre-flight checks succeed:

```bash
python3 installationdrone.py \
  --uri radio://0/80/2M/E7E7E7E7E8 \
  --traj_csv next_drone_full_trajectory_landing_frame_20260522_132342.csv \
  --fileroot data/installation_drone_logs \
  --logconfig logcfg.json \
  --filename install_test1 \
  --enable_mission_log \
  --enable_traj_log \
  --enable_velocity_log \
  --velocity_config_name mission_nd \
  --init_servo_ccw
```

## Official documentation

- [Crazyflie firmware building and flashing](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/)
- [Crazyflie out-of-tree builds](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/oot/)
- [Crazyflie Python library installation](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/install/)
- [Linux Crazyradio USB permissions](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/)
