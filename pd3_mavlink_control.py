#!/usr/bin/env python3
"""
PD #3 - MAVLink control of an ArduPilot SITL quadcopter with pymavlink.
NSDS VIP / ARGUS (C-UASC) - Simon Han

What it does, in order:
  1. Connects to ArduPilot SITL over MAVLink (UDP 14551, the "student API" port
     that MAVProxy in Drone_Sim_Docker forwards to).
  2. Reads and prints live telemetry (mode, armed state, GPS, altitude,
     position, speed, battery) and logs it to a CSV file.
  3. Waits until the autopilot reports it is ready (GPS 3D fix + pre-arm OK).
  4. Switches to GUIDED mode and arms the motors.
  5. Commands a takeoff to the target altitude.
  6. Flies guided-position waypoints (SET_POSITION_TARGET_GLOBAL_INT):
     WP1 north of home, WP2 north-east of home, then back over home.
  7. Switches to LAND, waits for touchdown and auto-disarm.

If anything goes wrong in the air (timeout, error, Ctrl+C), the script
commands LAND so the vehicle is never left flying without a controller.

Run (with `make run` already going in ~/Drone_Sim_Docker):
    python3 pd3_mavlink_control.py
    python3 pd3_mavlink_control.py --alt 10 --north 20 --east 15
"""

import argparse
import csv
import math
import sys
import time
from datetime import datetime

from pymavlink import mavutil

# ---------------------------------------------------------------- constants
DEFAULT_CONNECT = "udpin:127.0.0.1:14551"
EARTH_RADIUS_M = 6378137.0

# SYS_STATUS bit that ArduPilot sets when all pre-arm checks pass
PREARM_BIT = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK

# type_mask for SET_POSITION_TARGET_GLOBAL_INT: use position only
# (ignore velocity, acceleration, yaw and yaw-rate fields)
POSITION_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


class MissionAbort(Exception):
    """Raised when a step fails or times out."""


# ------------------------------------------------------------------ helpers
def offset_latlon(lat, lon, d_north_m, d_east_m):
    """Return a lat/lon moved d_north_m metres north and d_east_m metres east."""
    d_lat = d_north_m / EARTH_RADIUS_M
    d_lon = d_east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))
    return lat + math.degrees(d_lat), lon + math.degrees(d_lon)


def distance_m(lat1, lon1, lat2, lon2):
    """Horizontal distance between two lat/lon points (haversine), metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


# ------------------------------------------------------------------ vehicle
class Vehicle:
    """Thin wrapper around a pymavlink connection that keeps a telemetry state."""

    def __init__(self, connect_str, csv_path):
        log(f"Connecting to {connect_str} ...")
        self.m = mavutil.mavlink_connection(connect_str, source_system=250)
        self.state = {
            "mode": "?", "armed": False, "lat": None, "lon": None,
            "rel_alt": 0.0, "vx": 0.0, "vy": 0.0, "heading": 0.0,
            "fix": 0, "sats": 0, "volt": 0.0, "batt_pct": -1,
            "prearm_ok": False,
        }
        self.last_print = 0.0
        self.last_gcs_hb = 0.0
        self.csv_file = open(csv_path, "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["time_s", "phase", "mode", "armed", "lat", "lon",
                           "rel_alt_m", "groundspeed_mps", "heading_deg",
                           "gps_fix", "sats", "battery_v", "battery_pct"])
        self.t0 = time.time()
        self.phase = "connect"

    # -- connection -------------------------------------------------------
    def wait_for_autopilot(self, timeout=30):
        """Wait for a heartbeat from the autopilot (not from a GCS/MAVProxy)."""
        end = time.time() + timeout
        while time.time() < end:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb is None:
                continue
            if hb.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue
            self.m.target_system = hb.get_srcSystem()
            self.m.target_component = hb.get_srcComponent()
            log(f"Heartbeat from system {self.m.target_system} "
                f"component {self.m.target_component} "
                f"(MAV_TYPE {hb.type}, autopilot {hb.autopilot})")
            return
        raise MissionAbort("No heartbeat from the autopilot. Is `make run` running?")

    def request_telemetry(self):
        """Ask the autopilot to stream the messages we use, at fixed rates."""
        rates_hz = {
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2,
            mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS: 1,
        }
        for msg_id, hz in rates_hz.items():
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        log("Requested telemetry streams (position 5 Hz, status/GPS 2 Hz, battery 1 Hz)")

    # -- telemetry pump ---------------------------------------------------
    def update(self, wait=0.05):
        """Read every waiting MAVLink message and update self.state."""
        now = time.time()
        if now - self.last_gcs_hb > 1.0:  # behave like a well-mannered GCS
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_gcs_hb = now

        msg = self.m.recv_match(blocking=True, timeout=wait)
        while msg is not None:
            self._handle(msg)
            msg = self.m.recv_match(blocking=False)

        if now - self.last_print >= 1.0:
            self._print_and_log()
            self.last_print = now

    def _handle(self, msg):
        t = msg.get_type()
        s = self.state
        if t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system \
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
            s["mode"] = mavutil.mode_string_v10(msg)
            s["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif t == "GLOBAL_POSITION_INT":
            s["lat"] = msg.lat / 1e7
            s["lon"] = msg.lon / 1e7
            s["rel_alt"] = msg.relative_alt / 1000.0
            s["vx"], s["vy"] = msg.vx / 100.0, msg.vy / 100.0
            if msg.hdg != 65535:
                s["heading"] = msg.hdg / 100.0
        elif t == "GPS_RAW_INT":
            s["fix"], s["sats"] = msg.fix_type, msg.satellites_visible
        elif t == "SYS_STATUS":
            s["volt"] = msg.voltage_battery / 1000.0
            s["batt_pct"] = msg.battery_remaining
            s["prearm_ok"] = bool(msg.onboard_control_sensors_health & PREARM_BIT)
        elif t == "STATUSTEXT":
            log(f"  AUTOPILOT: {msg.text}")

    def _print_and_log(self):
        s = self.state
        gs = math.hypot(s["vx"], s["vy"])
        pos = f"{s['lat']:.6f},{s['lon']:.6f}" if s["lat"] is not None else "no position"
        print(f"    TLM | mode {s['mode']:<9} | {'ARMED   ' if s['armed'] else 'disarmed'} | "
              f"alt {s['rel_alt']:6.2f} m | gs {gs:4.1f} m/s | hdg {s['heading']:5.1f} | "
              f"GPS fix {s['fix']} sats {s['sats']:2d} | batt {s['volt']:.2f} V "
              f"{s['batt_pct']}% | {pos}", flush=True)
        self.csv.writerow([round(time.time() - self.t0, 2), self.phase, s["mode"],
                           int(s["armed"]), s["lat"], s["lon"], round(s["rel_alt"], 2),
                           round(gs, 2), s["heading"], s["fix"], s["sats"],
                           s["volt"], s["batt_pct"]])
        self.csv_file.flush()

    def wait_until(self, condition, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            self.update()
            if condition(self.state):
                return
        raise MissionAbort(f"Timed out after {timeout}s waiting for: {what}")

    # -- commands ---------------------------------------------------------
    def command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, timeout=5):
        """Send COMMAND_LONG and wait for its COMMAND_ACK. Returns True if accepted."""
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     cmd, 0, p1, p2, p3, p4, p5, p6, p7)
        end = time.time() + timeout
        while time.time() < end:
            ack = self.m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=0.5)
            if ack is None:
                continue
            if ack.get_type() == "STATUSTEXT":
                log(f"  AUTOPILOT: {ack.text}")
                continue
            if ack.command == cmd:
                ok = ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                name = mavutil.mavlink.enums["MAV_CMD"][cmd].name
                result = mavutil.mavlink.enums["MAV_RESULT"][ack.result].name
                log(f"  COMMAND_ACK {name}: {result}")
                return ok
        log("  (no COMMAND_ACK received)")
        return False

    def set_mode(self, mode_name, timeout=10):
        # ArduCopter: mode name -> custom_mode number
        mapping = {name: num for num, name in mavutil.mode_mapping_acm.items()}
        if mode_name not in mapping:
            raise MissionAbort(f"Unknown mode {mode_name}")
        log(f"Setting mode {mode_name}")
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[mode_name])
        self.wait_until(lambda s: s["mode"] == mode_name, timeout, f"mode {mode_name}")

    def arm(self, timeout=30):
        end = time.time() + timeout
        while time.time() < end:
            log("Sending ARM command")
            if self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1):
                self.wait_until(lambda s: s["armed"], 5, "armed")
                log("Motors ARMED")
                return
            for _ in range(40):  # read the autopilot's reason, then retry
                self.update()
        raise MissionAbort("Vehicle refused to arm (see AUTOPILOT messages above)")

    def takeoff(self, alt, timeout=45):
        log(f"Commanding TAKEOFF to {alt} m")
        if not self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt):
            raise MissionAbort("Takeoff rejected")
        self.wait_until(lambda s: s["rel_alt"] >= alt * 0.95, timeout,
                        f"altitude {alt} m")
        log(f"Reached takeoff altitude ({self.state['rel_alt']:.2f} m)")

    def goto(self, lat, lon, alt, name, tolerance=1.0, timeout=60):
        """Guided-position command: SET_POSITION_TARGET_GLOBAL_INT."""
        log(f"Guided waypoint {name}: lat {lat:.7f} lon {lon:.7f} alt {alt} m (relative)")

        def send():
            self.m.mav.set_position_target_global_int_send(
                0, self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                POSITION_ONLY_MASK, int(lat * 1e7), int(lon * 1e7), alt,
                0, 0, 0, 0, 0, 0, 0, 0)

        send()
        end = time.time() + timeout
        last_send = time.time()
        while time.time() < end:
            self.update()
            s = self.state
            if s["mode"] != "GUIDED":
                raise MissionAbort(f"Left GUIDED mode (now {s['mode']}) - fence or failsafe?")
            if time.time() - last_send > 2:  # re-send in case a packet was lost
                send()
                last_send = time.time()
            if s["lat"] is not None:
                d = distance_m(s["lat"], s["lon"], lat, lon)
                if d <= tolerance and abs(s["rel_alt"] - alt) < 1.0:
                    log(f"Reached {name} (error {d:.2f} m)")
                    for _ in range(20):  # short hover so it is visible in the video
                        self.update()
                    return
        raise MissionAbort(f"Timed out flying to {name}")

    def land(self, timeout=90):
        self.set_mode("LAND")
        self.wait_until(lambda s: not s["armed"], timeout, "touchdown and auto-disarm")
        log("Landed and DISARMED")

    def close(self):
        self.csv_file.close()


# --------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="PD #3 pymavlink SITL control demo")
    ap.add_argument("--connect", default=DEFAULT_CONNECT,
                    help=f"MAVLink connection string (default {DEFAULT_CONNECT})")
    ap.add_argument("--alt", type=float, default=10.0, help="flight altitude, m (default 10)")
    ap.add_argument("--north", type=float, default=20.0, help="WP1 metres north of home (default 20)")
    ap.add_argument("--east", type=float, default=15.0, help="WP2 metres east of WP1 (default 15)")
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"pd3_telemetry_{stamp}.csv"
    v = Vehicle(args.connect, csv_path)

    airborne = False
    try:
        print("=" * 78)
        print(" PD #3 - MAVLink control with pymavlink  |  ArduPilot SITL + Gazebo")
        print("=" * 78)

        # 1-2. connect and read telemetry
        v.wait_for_autopilot()
        v.request_telemetry()
        v.phase = "telemetry"
        log("Reading telemetry for 5 s ...")
        end = time.time() + 5
        while time.time() < end:
            v.update()

        # 3. pre-flight readiness
        v.phase = "preflight"
        log("Waiting for GPS 3D fix and pre-arm checks ...")
        v.wait_until(lambda s: s["fix"] >= 3 and s["lat"] is not None and s["prearm_ok"],
                     90, "GPS 3D fix + pre-arm OK")
        home_lat, home_lon = v.state["lat"], v.state["lon"]
        log(f"Ready. Home = {home_lat:.7f}, {home_lon:.7f}")

        # 4. GUIDED + arm
        v.phase = "arm"
        v.set_mode("GUIDED")
        v.arm()
        airborne = True

        # 5. takeoff
        v.phase = "takeoff"
        v.takeoff(args.alt)

        # 6. guided-position waypoints
        v.phase = "waypoints"
        wp1 = offset_latlon(home_lat, home_lon, args.north, 0)
        wp2 = offset_latlon(home_lat, home_lon, args.north, args.east)
        v.goto(*wp1, args.alt, f"WP1 ({args.north:.0f} m N)")
        v.goto(*wp2, args.alt, f"WP2 ({args.north:.0f} m N, {args.east:.0f} m E)")
        v.goto(home_lat, home_lon, args.alt, "HOME (above launch point)")

        # 7. land
        v.phase = "land"
        v.land()
        airborne = False

        print("=" * 78)
        log(f"MISSION COMPLETE. Telemetry log saved to {csv_path}")
        print("=" * 78)
        return 0

    except (MissionAbort, KeyboardInterrupt) as e:
        reason = "Ctrl+C" if isinstance(e, KeyboardInterrupt) else str(e)
        log(f"ABORT: {reason}")
        if airborne or v.state["armed"]:
            log("Safety action: commanding LAND")
            try:
                v.phase = "abort_land"
                v.land()
            except Exception as e2:  # noqa: BLE001
                log(f"LAND after abort did not complete: {e2}")
        return 1
    finally:
        v.close()


if __name__ == "__main__":
    sys.exit(main())
