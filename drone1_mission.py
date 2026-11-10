import math
import sys
import time

from pymavlink import mavutil

# ── CONFIG ────────────────────────────────────────────────────────────────────
CONNECTION     = 'udpin:0.0.0.0:14555'                       # MAVLink UDP port
WAYPOINTS_FILE = '/home/yuvraj/swarming_guided/drone_1.waypoints'
TAKEOFF_ALT    = 6.0    # metres AGL
WP_RADIUS      = 1.5    # metres — waypoint is "reached" within this distance
ALT_BAND       = 0.5    # metres — takeoff done when alt ≥ (TAKEOFF_ALT - ALT_BAND)
CRUISE_SPEED   = 2.0    # m/s
DRONE_ID       = 'D1'
# ─────────────────────────────────────────────────────────────────────────────

# ArduCopter custom_mode values
GUIDED_MODE = 4   # GUIDED
RTL_MODE    = 6   # RTL  (for reference)

# SET_POSITION_TARGET_GLOBAL_INT type_mask: ignore velocity / accel / yaw
_POS_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE    |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE    |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE    |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE    |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE    |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE    |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE   |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


# ── HELPERS ───────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    print(f'[{DRONE_ID}] {msg}', flush=True)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Horizontal distance in metres between two GPS coordinates."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2.0 * R * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def parse_waypoints(path: str) -> list:
    """
    Parse a QGC WPL 110 waypoints file.
    Returns [(lat, lon, alt), …] — home, takeoff, and RTL entries are skipped.
    """
    waypoints = []
    with open(path) as f:
        lines = f.readlines()

    if not lines or not lines[0].startswith('QGC WPL'):
        raise ValueError(f'Not a valid QGC WPL file: {path}')

    for line in lines[1:]:                  # skip 'QGC WPL 110' header
        parts = line.strip().split('\t')
        if len(parts) < 11:
            continue
        current = int(parts[1])             # 1 = home row
        cmd     = int(parts[3])             # MAV_CMD
        if current == 1:                    # home — skip
            continue
        if cmd == 22:                       # NAV_TAKEOFF — skip (we do it ourselves)
            continue
        if cmd == 20:                       # NAV_RETURN_TO_LAUNCH — skip
            continue
        lat = float(parts[8])
        lon = float(parts[9])
        alt = float(parts[10])
        waypoints.append((lat, lon, alt))

    return waypoints


# ── MAIN STEPS ────────────────────────────────────────────────────────────────

def connect() -> mavutil.mavlink_connection:
    log(f'Connecting → {CONNECTION}')
    master = mavutil.mavlink_connection(CONNECTION)
    master.wait_heartbeat()
    log(f'Heartbeat OK  system={master.target_system}  component={master.target_component}')
    return master


def wait_guided_and_armed(master) -> None:
    """
    Block until the RC operator arms the drone AND switches to GUIDED mode.
    The script does NOT arm or change mode — that is done via RC.
    """
    log('Waiting for RC to ARM + switch to GUIDED …')
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb is None:
            continue
        armed  = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        guided = (hb.custom_mode == GUIDED_MODE)
        print(f'\r  [{DRONE_ID}]  armed={armed}  mode={hb.custom_mode} (need GUIDED={GUIDED_MODE})   ',
              end='', flush=True)
        if armed and guided:
            print()
            log('Arm + GUIDED confirmed — starting mission')
            return
        time.sleep(0.2)


def send_takeoff_and_wait(master, target_alt: float) -> None:
    """Continuously sends the takeoff command until target altitude is reached."""
    log(f'Initiating takeoff loop to {target_alt} m …')
    last_cmd_time = 0

    while True:
        now = time.time()
        # Send the takeoff command every 2 seconds to ensure it is received
        if now - last_cmd_time > 2.0:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,            # confirmation
                0,            # param1: min pitch (deg)
                0, 0, 0,      # param2-4: unused
                0, 0,         # param5-6: lat/lon  (0 = use current position)
                target_alt    # param7: altitude (m AGL)
            )
            last_cmd_time = now

        # Wait up to 1 second for a position message
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg is None:
            continue
            
        rel_alt = msg.relative_alt / 1000.0    # mm → m
        print(f'\r  [{DRONE_ID}]  altitude = {rel_alt:.1f} m  /  {target_alt} m   ',
              end='', flush=True)
              
        # Break the loop if we have reached the target altitude band
        if rel_alt >= target_alt - ALT_BAND:
            print()
            log(f'Takeoff complete  ({rel_alt:.1f} m)')
            return


def set_speed(master, speed: float) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        1,       # speed type: 0=airspeed  1=groundspeed
        speed,   # speed (m/s)
        -1,      # throttle (-1 = no change)
        0, 0, 0, 0
    )
    log(f'Speed set to {speed} m/s')


def goto_waypoint(master, lat: float, lon: float, alt: float) -> None:
    """Send drone to target and block until within WP_RADIUS metres."""
    master.mav.set_position_target_global_int_send(
        0,                                                # time_boot_ms (ignored)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        _POS_ONLY_MASK,
        int(lat * 1e7),   # lat  (degE7)
        int(lon * 1e7),   # lon  (degE7)
        alt,              # alt  (m AGL)
        0, 0, 0,          # vx vy vz
        0, 0, 0,          # ax ay az
        0, 0              # yaw  yaw_rate
    )
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg is None:
            continue
        cur_lat = msg.lat / 1e7
        cur_lon = msg.lon / 1e7
        dist    = haversine_m(cur_lat, cur_lon, lat, lon)
        print(f'\r  [{DRONE_ID}]  target=({lat:.6f}, {lon:.6f})  dist={dist:.1f} m   ',
              end='', flush=True)
        if dist <= WP_RADIUS:
            print()
            return


def send_rtl(master) -> None:
    log('Sending RTL …')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )


# ── ENTRY POINT ───────────────────────────────────────────────────────────────

def main():
    master = connect()

    # ── Step 1: wait for RC arm + GUIDED ────────────────────────────────────
    wait_guided_and_armed(master)

    # ── Step 2: takeoff ─────────────────────────────────────────────────────
    send_takeoff_and_wait(master, TAKEOFF_ALT)
    set_speed(master, CRUISE_SPEED)

    # ── Step 3: load waypoints ──────────────────────────────────────────────
    try:
        waypoints = parse_waypoints(WAYPOINTS_FILE)
    except FileNotFoundError:
        log(f'ERROR: waypoints file not found: {WAYPOINTS_FILE}')
        sys.exit(1)

    if not waypoints:
        log('ERROR: no waypoints found in file — aborting')
        sys.exit(1)

    log(f'Loaded {len(waypoints)} waypoints from {WAYPOINTS_FILE}')

    # ── Step 4: execute waypoints one by one ─────────────────────────────────
    for i, (lat, lon, alt) in enumerate(waypoints):
        log(f'→ WP {i + 1}/{len(waypoints)}   ({lat:.7f}, {lon:.7f}, {alt:.1f} m)')
        goto_waypoint(master, lat, lon, alt)
        log(f'✓ WP {i + 1} reached')

    # ── Step 5: RTL ──────────────────────────────────────────────────────────
    send_rtl(master)
    log('Mission complete — returning home')


if __name__ == '__main__':
    main()
