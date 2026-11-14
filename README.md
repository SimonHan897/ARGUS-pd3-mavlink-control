# PD #3: MAVLink control with pymavlink

NSDS VIP / ARGUS (C-UASC), Simon Han

**Completion criteria:** a program that connects to SITL, reads telemetry, arms the vehicle, commands takeoff, sends guided-position waypoint commands and lands safely. Record a video of the run and upload it to Brightspace.

## How to run

Terminal 1 starts the simulator (Gazebo, ArduPilot SITL and MAVProxy):

```bash
cd ~/Drone_Sim_Docker
make run
```

Optionally, open QGroundControl too. It connects on UDP 14550 and shows the map view.

Wait until the Iris drone appears in Gazebo and QGC says **Ready to Fly**. Then use terminal 2:

```bash
cd ~/ros2_ws/pd3_mavlink
./run_pd3.sh                       # defaults: 10 m altitude, WP1 20 m N, WP2 +15 m E
./run_pd3.sh --alt 15 --north 25   # options
```

Close any MAVProxy console that is using port 14551 first. Only one program can listen on that port.

## What talks to what

```
 Gazebo Harmonic (physics + 3D view)
        ▲  JSON over UDP (motor outputs ↔ sensor data)
        ▼
 ArduPilot Copter 4.6.3 SITL (the real flight-control code)
        ▲  MAVLink over TCP 5760
        ▼
 MAVProxy (router)  ── UDP 14550 ──► QGroundControl (ground station)
        └────────── UDP 14551 ──► pd3_mavlink_control.py (pymavlink)
```

All of this runs inside the `ardupilot-gz-sim` Docker container, except QGroundControl and this script, which run on the host. The container uses host networking, so everything is reachable at `127.0.0.1`.

## What the program does (MAVLink messages used)

| Step | MAVLink message / command | Success check |
|---|---|---|
| Connect | wait for `HEARTBEAT` from the autopilot (GCS heartbeats are ignored) | system/component IDs found |
| Telemetry | `MAV_CMD_SET_MESSAGE_INTERVAL` → `GLOBAL_POSITION_INT`, `GPS_RAW_INT`, `SYS_STATUS`, `HEARTBEAT` | 1 Hz `TLM` line printed and logged to CSV |
| Pre-flight | `GPS_RAW_INT.fix_type ≥ 3` and the `SYS_STATUS` pre-arm-check bit | ready to arm |
| GUIDED mode | `MAV_CMD_DO_SET_MODE` (custom mode 4) | `COMMAND_ACK` + heartbeat shows GUIDED |
| Arm | `MAV_CMD_COMPONENT_ARM_DISARM` (param1 = 1) | `COMMAND_ACK` + armed flag in heartbeat |
| Takeoff | `MAV_CMD_NAV_TAKEOFF` (param7 = altitude) | relative altitude ≥ 95 % of target |
| Waypoints | `SET_POSITION_TARGET_GLOBAL_INT` (position-only type mask, relative-altitude frame): WP1 → WP2 → back over home | within 1 m horizontally and 1 m vertically |
| Land | `MAV_CMD_DO_SET_MODE` → LAND | vehicle touches down and auto-disarms |

**Safety:** every step has a timeout. If a step times out, the vehicle leaves GUIDED (for example, a fence breach), or you press Ctrl+C, the script commands **LAND**. The vehicle is never left in the air without control. The script also sends a 1 Hz GCS heartbeat and prints every autopilot `STATUSTEXT`, so pre-arm failures show up on screen.

Each run writes `pd3_telemetry_<date>_<time>.csv` (time, phase, mode, armed, lat/lon, altitude, ground speed, heading, GPS, battery) for the report.

## Video outline (about 2–3 minutes)

1. Show the terminal running `make run`, Gazebo with the Iris, and QGC (optional). Explain the diagram above.
2. Briefly show the code: the connection string, the command functions and the mission sequence in `main()`.
3. Run `./run_pd3.sh`. Point out the heartbeat, the telemetry lines, the GUIDED/ARM/TAKEOFF ACKs, and the drone climbing in Gazebo.
4. Show the waypoint legs in QGC or Gazebo, then LAND and DISARMED.
5. Open the CSV log, then upload the video to Brightspace.

## Troubleshooting

- **No heartbeat:** the sim isn't running, or another program (a MAVProxy console) is holding UDP 14551.
- **"PreArm: Battery failsafe":** usually left over from failsafe testing. Restart the sim. If it keeps happening, run `WIPE_PARAMS=1 make run` once to reset the parameters.
- **"Left GUIDED mode":** a fence or another failsafe triggered. Use smaller `--north`/`--east` values.
