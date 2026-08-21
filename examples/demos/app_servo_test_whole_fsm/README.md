# Sensor Placement Demo - Crazyflie

This folder contains the out-of-tree Crazyflie app used for the sensor-placement / landing-frame installation demo. It receives a pre-generated XY trajectory over CRTP, runs the onboard mission FSM, controls the servo, and logs the drone state during the mission.

The main files are:

- `src/app_main.c` - app loop, state estimate + mission FSM bridge
- `src/mission_fsm.c` - state machine for hover, trajectory follow, contact detection, servo trigger, landing
- `src/servo_test.c` - servo control logic
- `installationdrone.py` - Python script that sends the trajectory and controls the flight sequence
- `logcfg.json` - logging configuration for flight data
- `data/installation_drone_logs/` - recorded logs

This README is written as a step-by-step checklist so a new user can build, flash, and run the demo without guessing.

---

## 1. Hardware and software prerequisites

### Required hardware

- Crazyflie 2.x or Crazyflie 2.1 Brushless
- Crazyradio or Crazyradio PA
- Servo and actuator connected to the board as defined in the firmware app
- A stable test area and a proper takeoff/landing area
- USB cable for debugging if needed

### Required software

Follow the official Crazyflie build and flashing guide first:

- https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/

That page is the required setup reference for the toolchain and the official Crazyflie client / flashing workflow for your platform.

> If you are on Windows, use WSL and follow the official Ubuntu/WSL setup in that guide. Do not skip the official build-and-flash instructions before trying to run this demo.

---

## 2. Clone the repo and fetch submodules

From a Linux terminal:

```bash
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware
```

If you already cloned without `--recursive`:

```bash
git submodule update --init --recursive
```

---

## 3. Build the firmware

This demo is an out-of-tree app. Build the main firmware first, then build the demo app.

### 3.1 Configure the correct target

For a Crazyflie 2.1 Brushless board, use:

```bash
cd crazyflie-firmware
make cf21bl_defconfig
```

If you are using a standard Crazyflie 2.x, use:

```bash
make cf2_defconfig
```

### 3.2 Build the firmware

```bash
make -j$(nproc)
```

This generates the firmware binary in the firmware build directory, usually under:

```text
crazyflie-firmware/build/
```

### 3.3 Build this out-of-tree demo app

Go to the demo directory:

```bash
cd examples/demos/app_servo_test_whole_fsm
make clean
make
```

This app uses the `app-config` file in this folder and builds the firmware as an app layer extension. You should see the build finish successfully. If the build output is not obvious, find the binary with:

```bash
find .. -name "cf21bl*.bin" -o -name "cf2*.bin"
```

---

## 4. Flash the firmware to the Crazyflie

There are two common ways.

### Option A: flash using Crazyradio and the wireless bootloader

1. Turn the Crazyflie off.
2. Put it into bootloader mode by holding the power button for about 3 seconds until the blue LEDs blink together.
3. From the firmware root, run:

```bash
cd crazyflie-firmware
make cload
```

This tries to flash the current compiled firmware to the Crazyflie in bootloader mode.

If you want to specify the URI directly:

```bash
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E8" make cload
```

If you want to flash the generated binary directly with the loader:

```bash
cfloader flash build/cf21bl.bin stm32-fw -w radio://0/80/2M/E7E7E7E7E8
```

> Replace the URI with your own Crazyflie radio address if needed.

### Option B: flash via ST-Link / debug adapter

If you have a debug adapter:

```bash
cd crazyflie-firmware
make flash
```

This requires OpenOCD and an ST-Link-compatible debugger to be installed and configured.

---

## 5. Verify the app is running

After flashing, power on the Crazyflie and connect to it from your terminal.

A quick sanity check is to use the Python client and connect to the drone URI:

```bash
cd crazyflie-firmware/examples/demos/app_servo_test_whole_fsm
python3 -c "import cflib.crtp; cflib.crtp.init_drivers(); print('cflib ok')"
```

Then you can run the mission script described below.

---

## 6. Run the sensor placement / installation demo

This folder includes a Python runner named `installationdrone.py`.

The script:

- resets mission flags
- optionally moves the servo to the CCW starting angle
- arms the drone
- sends the landing-frame trajectory over CRTP
- starts the mission
- logs flight data to CSV files

### 6.1 Choose a trajectory CSV

The demo already includes a few trajectory files in this folder such as:

```text
next_drone_full_trajectory_landing_frame_20260522_132342.csv
```

You can also use one from `data/installation_drone_logs/` if you already have a generated result.

### 6.2 Launch the mission

Run the script from this demo directory:

```bash
cd crazyflie-firmware/examples/demos/app_servo_test_whole_fsm
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

This command will:

1. connect to the Crazyflie
2. reset the trajectory flags
3. initialize the servo to the CCW starting position
4. reset the estimator
5. arm the drone
6. send the trajectory points over CRTP
7. start the mission
8. log the mission to CSV

### 6.3 Manual emergency stop

If you need to stop the mission during flight, press `Ctrl+C`.

The script will request an onboard landing and then disarm on exit.

---

## 7. Useful optional commands

### Dry run only (preview the path without connecting)

```bash
python3 installationdrone.py \
  --traj_csv next_drone_full_trajectory_landing_frame_20260522_132342.csv \
  --dry_run
```

### Force a collision for test logic

```bash
python3 installationdrone.py \
  --uri radio://0/80/2M/E7E7E7E7E8 \
  --traj_csv next_drone_full_trajectory_landing_frame_20260522_132342.csv \
  --force-collision-after 8 \
  --init_servo_ccw
```

This is useful for testing the contact-detection state machine without needing a real external collision.

---

## 8. What the firmware is doing during the mission

The app uses the onboard mission FSM to execute these phases:

```text
IDLE -> HOVERING -> FOLLOWING -> FLY_FORWARD -> CONTACT -> SERVO_CW -> FLY_BACKWARD -> LANDING -> FINISHED
```

The Python side sends trajectory points as CRTP packets. A final `NaN, NaN` packet marks the end of the trajectory. Then the app sets `traj.start=1` to start the mission.

The servo motion is controlled through the `servotest.cmd` parameter:

- `1` = CCW
- `2` = CW

This is handled by `servo_test.c` and triggered during the mission state machine.

---

## 9. Where to check logs

The logs are written to:

```text
crazyflie-firmware/examples/demos/app_servo_test_whole_fsm/data/installation_drone_logs/
```

The default log config is in:

```text
crazyflie-firmware/examples/demos/app_servo_test_whole_fsm/logcfg.json
```

If you want to inspect or analyze the flight data, use the Jupyter notebooks in this folder such as the tracking-analysis notebooks.

---

## 10. Common troubleshooting

### Build fails because the compiler is missing

```bash
sudo apt install -y gcc-arm-none-eabi make
```

### Wrong board configuration

The app configuration contains:

```text
CONFIG_PLATFORM_CF21BL=y
```

This implies the demo is meant for Crazyflie 2.1 Brushless. If you are using another platform, change the configuration accordingly or build against the correct target.

### The drone does not flash

- Make sure the Crazyflie is in bootloader mode
- Make sure only one Crazyflie is in range
- Make sure the URI is correct
- Check that the firmware has been built successfully

### The trajectory is not executing

- Confirm the app is flashed and running
- Ensure `traj.start` is set to `1` after sending all trajectory points
- Confirm the final packet is `NaN, NaN`
- Check the console output for `CRTP trajectory receiver registered` messages

### The servo does not move

- Confirm the servo is wired correctly to TX2 / PA2
- Confirm the app is built for the correct board
- Confirm the startup command `--init_servo_ccw` is used

---

## 11. Fastest “do it now” sequence

If you just want the shortest path to get this running:

```bash
cd crazyflie-firmware
make cf21bl_defconfig
make -j$(nproc)
cd examples/demos/app_servo_test_whole_fsm
make clean
make
```

Put the drone in bootloader mode and run:

```bash
cd ../..
make cload
```

Then run the demo:

```bash
cd examples/demos/app_servo_test_whole_fsm
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

This is the standard path for running the sensor placement / landing-frame mission end-to-end.

---

## 12. Summary

The workflow is:

1. install toolchain
2. configure/build the firmware
3. flash the firmware to the Crazyflie
4. run the Python mission script
5. monitor the logs and trajectory execution

If you follow these steps in order, the demo should run reliably in a normal Crazyflie lab setup.
