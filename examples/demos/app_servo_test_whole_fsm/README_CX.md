# CRTP trajectory tracking version

Protocol:
- Python sends each trajectory point as one CRTP packet on port `0x0F`.
- Payload is `<ff>`: `x, y`.
- A final `NaN, NaN` packet marks the end of the trajectory.

Onboard state sequence:
`IDLE -> HOVERING(5s) -> FOLLOWING -> FLY_FORWARD -> CONTACT -> SERVO_CW -> FLY_BACKWARD -> LANDING -> FINISHED`

Run:
```bash
python trajectory_client.py --uri radio://0/80/2M/E7E7E7E7E8 --example 2
```

Manual contact test:
```bash
python trajectory_client.py --uri radio://0/80/2M/E7E7E7E7E8 --example 2 --force-collision-after 8
```

Run:
python installationdrone.py \
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