#!/usr/bin/env python3
"""Generate a QGC WPL 110 geofence file (square, centred on SITL home).

Usage: python3 gen_geofence.py [lat] [lon] [area_m2] [outfile]
Defaults: ArduPilot SITL default home (CMAC), 4000 m^2, mission_sitl.txt
"""
import math, sys

LAT   = float(sys.argv[1]) if len(sys.argv) > 1 else -35.363261
LON   = float(sys.argv[2]) if len(sys.argv) > 2 else 149.165230
AREA  = float(sys.argv[3]) if len(sys.argv) > 3 else 4000.0
OUT   = sys.argv[4] if len(sys.argv) > 4 else "mission_sitl.txt"
ALT   = 100.0

R = 6_371_000.0
half = math.sqrt(AREA) / 2.0                       # 31.6228 m for 4000 m^2
dlat = math.degrees(half / R)
dlon = math.degrees(half / (R * math.cos(math.radians(LAT))))

# CCW square corners: SW, SE, NE, NW
corners = [(LAT - dlat, LON - dlon),
           (LAT - dlat, LON + dlon),
           (LAT + dlat, LON + dlon),
           (LAT + dlat, LON - dlon)]

with open(OUT, "w") as f:
    f.write("QGC WPL 110\n")
    for i, (la, lo) in enumerate(corners):
        cur   = 1 if i == 0 else 0
        frame = 0 if i == 0 else 3
        f.write(f"{i}\t{cur}\t{frame}\t16\t0.00000000\t0.00000000\t0.00000000\t"
                f"0.00000000\t{la:.8f}\t{lo:.8f}\t{ALT:.6f}\t1\n")

print(f"{OUT}: {len(corners)} vertices, side {2*half:.3f} m, area {(2*half)**2:.1f} m^2")
print(f"centre {LAT:.7f}, {LON:.7f}")
