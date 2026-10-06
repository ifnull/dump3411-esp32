#!/usr/bin/env python3
"""Check the C tracker against dump3411's tracker.py on the same timed input.

Each scenario is a script of timestamped tracker updates, sweeps and
snapshots (format in host/tracker_sim.c). The script runs through the C
tracker (build-host/tracker_sim) and through dump3411's Tracker with its
clock injected, and every snapshot must match: same drones in the same
order, same fields, same on_change / on_expire calls, and numbers equal up
to tracker.py's own rounding.

Scenarios are a hand-written set covering the rules in tracker.py's
docstring, plus seeded random streams (several drones, rotating MACs,
transports, partial fields, silences long enough to expire).

    cmake -S host -B build-host && cmake --build build-host
    python3 tests/tracker/run_tracker_parity.py [--dump3411 PATH] [--random N] [--seed S]

Needs a dump3411 whose Tracker accepts clock= (injectable clock).
"""

import argparse
import inspect
import json
import os
import random
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TRACKER_SIM = ROOT / "build-host" / "tracker_sim"
TTL_MS = 60_000

# tracker.py rounds on output; C prints unrounded. Allow half a rounding step.
TOLERANCE = {
    "seen": 0.051, "seen_pos": 0.051, "self_id_seen": 0.051,
    "lat": 1e-7, "lon": 1e-7,
    "alt_geom_ft": 0.051, "agl_ft": 0.051, "gs": 0.051, "track": 0.051,
    "alt_takeoff_ft": 0.051,
    "geom_rate": 0.51,
}
EXACT = {
    "id", "id_type", "message_count", "ua_type", "rssi", "rid_source",
    "rid_sources", "self_id", "operator", "location_type",
}
LOCATION_TYPE = {0: "takeoff", 1: "live_gnss", 2: "fixed", 3: "reserved"}


# -- scenarios --------------------------------------------------------------

def hexs(text: bytes) -> str:
    return text.hex() if text else "-"


def handwritten():
    """Each rule in tracker.py's docstring, exercised on purpose."""
    a, a2, b = "a1a1a1a1a1a1", "a2a2a2a2a2a2", "b1b1b1b1b1b1"
    drone_a = hexs(b"DRONE-A")
    yield "identity, MAC rotation, transports", [
        "1000 L a1a1a1a1a1a1 ble -70 30.1 -97.1 300 50 5 90 0",   # orphan: dropped
        f"1100 B {a} ble -70 1 15 {drone_a}",                        # ua 15 unknown -> none
        f"1200 L {a} ble5 -71 30.1 -97.1 300 50 5 90 0.5",
        f"1300 B {a} ble -69 3 2 {drone_a}",                         # ua filled, id_type stays
        f"1400 B {a} ble -69 3 4 {drone_a}",                         # ua write-once
        "1500 P",
        f"2000 B {a2} wifi_beacon -50 1 2 {drone_a}",                # same drone, new MAC
        f"2100 L {a2} wifi_beacon - 30.2 -97.2 - - 70.5 271 -3",    # rssi unknown, no altitudes
        f"2200 L {a} ble -72 30.3 -97.3 310 55 6 91 1",              # old MAC still mapped
        "2300 P",
    ]
    yield "system and string fields", [
        f"1000 B {b} wifi_nan -60 2 1 {hexs(b'DRONE-B')}",
        f"1100 S {b} wifi_nan -60 30.5 -97.5 1 250",
        f"1200 S {b} wifi_nan -61 - - - 260",                        # keeps operator position
        f"1300 S {b} wifi_nan -62 30.6 -97.6 3 -",                   # keeps altitude, reserved type
        f"1400 I {b} wifi_nan -60 {hexs(b'Survey')}",
        f"1500 I {b} wifi_nan -60 -",                                # empty: ignored
        f"1600 O {b} wifi_nan -60 {hexs(b'OP-123')}",
        f"1700 O {b} wifi_nan -60 -",                                # empty: ignored
        f"1800 I {b} wifi_nan -60 {hexs(bytes([0x41, 0xFF, 0x42]))}",  # non-ASCII byte
        "1900 P",
    ]
    yield "expiry, order and recreation", [
        f"0 B {a} ble -70 1 2 {drone_a}",
        f"10000 B {b} ble -70 1 2 {hexs(b'DRONE-B')}",
        "60000 W",                                                   # A is exactly ttl old: kept
        "60000 P",
        "60001 W",                                                   # A expires
        "60002 P",
        "61000 L a1a1a1a1a1a1 ble -70 30 -97 300 50 5 90 0",        # A's MAC unmapped: dropped
        f"62000 B {a} ble5 -70 1 2 {drone_a}",                       # A again, after B now
        "62000 P",
        "70001 W",                                                   # B expires
        "70002 P",
    ]


def random_scenario(rng: random.Random, minutes: int = 6):
    """A noisy stream of several drones; updates on even ms, sweeps on odd ms
    (so no drone is ever exactly ttl old at a sweep, where float time in
    tracker.py and integer time in C may round differently)."""
    sources = ["ble", "ble5", "wifi_beacon", "wifi_nan"]
    drones = []
    for i in range(rng.randint(1, 6)):
        uas = f"RND-{i}-{rng.randrange(10**6):06d}".encode()
        if rng.random() < 0.1:
            uas = uas[:4] + b"\xc3" + uas[5:]
        macs = ["%012x" % rng.randrange(1 << 48) for _ in range(rng.randint(1, 3))]
        drones.append({"uas": uas, "macs": macs, "active": rng.random() < 0.8,
                       "id_type": rng.randint(0, 6), "ua_type": rng.choice([0, 2, 5, 14, 15]),
                       "lat": rng.uniform(-60, 60), "lon": rng.uniform(-170, 170)})
    lines = []
    t = 0
    end = minutes * 60_000
    next_sweep, next_snap = 1001, 5000
    while t < end:
        t += 2 * rng.randint(20, 250)                 # even ms
        while next_sweep < t:
            lines.append(f"{next_sweep} W")
            next_sweep += 1000
        while next_snap < t:
            lines.append(f"{next_snap} P")
            next_snap += rng.choice([2000, 5000, 9000])
        d = rng.choice(drones)
        if rng.random() < 0.02:
            d["active"] = not d["active"]             # go quiet long enough to expire, or return
        if not d["active"]:
            continue
        mac = rng.choice(d["macs"]) if rng.random() < 0.9 else "%012x" % rng.randrange(1 << 48)
        src = rng.choice(sources)
        rssi = str(rng.randint(-95, -30)) if rng.random() < 0.95 else "-"
        kind = rng.choices("BLSIO", weights=[3, 6, 2, 1, 1])[0]
        head = f"{t} {kind} {mac} {src} {rssi}"
        if kind == "B":
            id_type = d["id_type"] if rng.random() < 0.9 else rng.randint(0, 15)
            ua_type = d["ua_type"] if rng.random() < 0.7 else rng.randint(0, 15)
            lines.append(f"{head} {id_type} {ua_type} {d['uas'].hex()}")
        elif kind == "L":
            d["lat"] += rng.uniform(-1e-4, 1e-4)
            d["lon"] += rng.uniform(-1e-4, 1e-4)
            alt = repr(round(rng.uniform(-100, 3000) * 2) / 2) if rng.random() < 0.85 else "-"
            agl = repr(round(rng.uniform(0, 400) * 2) / 2) if rng.random() < 0.85 else "-"
            speed = repr(rng.randint(0, 255) * 0.25)
            heading = repr(float(rng.randint(0, 359)))
            vs = repr(rng.randint(-126, 126) * 0.5)
            lines.append(f"{head} {round(d['lat'], 7)!r} {round(d['lon'], 7)!r} {alt} {agl} {speed} {heading} {vs}")
        elif kind == "S":
            if rng.random() < 0.8:
                op = f"{round(d['lat'] + 1e-3, 7)!r} {round(d['lon'] - 1e-3, 7)!r} {rng.randint(0, 3)}"
            else:
                op = "- - -"
            alt = repr(round(rng.uniform(0, 500) * 2) / 2) if rng.random() < 0.8 else "-"
            lines.append(f"{head} {op} {alt}")
        elif kind == "I":
            text = rng.choice([b"", b"Photography", b"Search and Rescue", b"Inspect\x01"])
            lines.append(f"{head} {hexs(text)}")
        else:
            text = rng.choice([b"", b"FIN87astrdge12k8", b"OP-1"])
            lines.append(f"{head} {hexs(text)}")
    lines.append(f"{max(t, end) + 1} P")
    return lines


# -- running ----------------------------------------------------------------

def run_c(lines):
    run = subprocess.run([str(TRACKER_SIM), str(TTL_MS)], input="\n".join(lines) + "\n",
                         capture_output=True, text=True)
    if run.returncode != 0:
        raise RuntimeError(f"tracker_sim failed: {run.stderr}")
    return [json.loads(x) for x in run.stdout.splitlines()]


def run_python(tracker_mod, lines):
    now = [0.0]
    changes, expired = [], []
    t = tracker_mod.Tracker(ttl_seconds=TTL_MS / 1000.0,
                            on_change=lambda uas, row: changes.append(uas),
                            on_expire=lambda uas: expired.append(uas),
                            clock=lambda: now[0], wall_clock=lambda: 0.0)

    def text(h):
        return "" if h == "-" else bytes.fromhex(h).decode("ascii", errors="replace")

    def num(s):
        return None if s == "-" else float(s)

    def mac(h):
        return ":".join(h[i:i + 2] for i in range(0, 12, 2))

    out = []
    for line in lines:
        f = line.split()
        now[0] = int(f[0]) / 1000.0
        op = f[1]
        if op == "W":
            t._sweep_once()
            continue
        if op == "P":
            snap = t.snapshot()
            out.append({"t": int(f[0]), "messages": snap["messages"], "drones": snap["drones"],
                        "changes": list(changes), "expired": list(expired)})
            changes.clear()
            expired.clear()
            continue
        common = {"mac": mac(f[2]), "rid_source": f[3],
                  "rssi": None if f[4] == "-" else int(f[4])}
        if op == "B":
            t.update_basic_id(uas_id=text(f[7]), id_type_raw=int(f[5]), ua_type_raw=int(f[6]), **common)
        elif op == "L":
            t.update_location(lat=float(f[5]), lon=float(f[6]), alt_geo_m=num(f[7]),
                              height_agl_m=num(f[8]), gs_mps=float(f[9]),
                              heading_deg=float(f[10]), vspeed_mps=float(f[11]), **common)
        elif op == "S":
            loc = None if f[7] == "-" else LOCATION_TYPE[int(f[7])]
            t.update_system(op_lat=num(f[5]), op_lon=num(f[6]), alt_takeoff_m=num(f[8]),
                            op_location_type=loc, **common)
        elif op == "I":
            t.update_self_id(description=text(f[5]), **common)
        elif op == "O":
            t.update_operator_id(operator_id=text(f[5]), **common)
    return out


def compare(c, py, path, diffs):
    if isinstance(c, dict) and isinstance(py, dict):
        for key in sorted(set(c) | set(py)):
            where = f"{path}.{key}"
            if key not in c:
                diffs.append(f"{where}: only Python has it ({py[key]!r})")
            elif key not in py:
                diffs.append(f"{where}: only C has it ({c[key]!r})")
            elif key in TOLERANCE:
                if abs(c[key] - py[key]) > TOLERANCE[key]:
                    diffs.append(f"{where}: C {c[key]} vs Python {py[key]}")
            elif key in EXACT or isinstance(c[key], (dict, list)):
                compare(c[key], py[key], where, diffs)
            else:
                diffs.append(f"{where}: unrecognized field, no comparison rule")
    elif isinstance(c, list) and isinstance(py, list):
        if len(c) != len(py):
            diffs.append(f"{path}: C has {len(c)} items, Python {len(py)}")
        for i, (a, b) in enumerate(zip(c, py)):
            compare(a, b, f"{path}[{i}]", diffs)
    elif c != py:
        diffs.append(f"{path}: C {c!r} vs Python {py!r}")


def check(name, lines, tracker_mod, verbose):
    c_snaps, py_snaps = run_c(lines), run_python(tracker_mod, lines)
    diffs = []
    if len(c_snaps) != len(py_snaps):
        diffs.append(f"C printed {len(c_snaps)} snapshots, Python {len(py_snaps)}")
    for c, py in zip(c_snaps, py_snaps):
        for key in ("messages", "changes", "expired"):
            compare(c[key], py[key], f"t={c['t']}.{key}", diffs)
        compare(c["drones"], py["drones"], f"t={c['t']}.drones", diffs)
    drones_seen = max((len(s["drones"]) for s in py_snaps), default=0)
    expiries = sum(len(s["expired"]) for s in py_snaps)
    if diffs:
        print(f"FAIL {name}")
        for d in diffs[:20]:
            print(f"    {d}")
        if len(diffs) > 20:
            print(f"    ... {len(diffs) - 20} more")
    elif verbose:
        print(f"ok   {name}: {len(lines)} events, {len(py_snaps)} snapshots, "
              f"up to {drones_seen} drones, {expiries} expiries")
    return not diffs, len(lines), expiries


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    default = os.environ.get("DUMP3411_DIR", str(ROOT.parent / "dump3411"))
    parser.add_argument("--dump3411", default=default, help="dump3411 checkout (default: %(default)s)")
    parser.add_argument("--random", type=int, default=50, help="random scenarios (default: %(default)s)")
    parser.add_argument("--seed", type=int, default=3411, help="random seed (default: %(default)s)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    dump3411_dir = Path(args.dump3411).resolve()
    sys.path.insert(0, str(dump3411_dir))
    import tracker as tracker_mod  # noqa: E402
    if "clock" not in inspect.signature(tracker_mod.Tracker.__init__).parameters:
        print(f"{dump3411_dir}/tracker.py has no injectable clock; "
              f"use a dump3411 with 'refactor(tracker): accept an injectable clock'")
        return 2
    if not TRACKER_SIM.exists():
        print(f"{TRACKER_SIM} missing; build it: cmake -S host -B build-host && cmake --build build-host")
        return 2

    rev = subprocess.run(["git", "-C", str(dump3411_dir), "describe", "--always", "--dirty"],
                         capture_output=True, text=True).stdout.strip() or "not a git checkout"
    print(f"dump3411 tracker: {dump3411_dir} @ {rev}")

    rng = random.Random(args.seed)
    scenarios = list(handwritten())
    scenarios += [(f"random #{i} (seed {args.seed})", random_scenario(rng)) for i in range(args.random)]

    passed = events = expiries = 0
    for name, lines in scenarios:
        ok, n, e = check(name, lines, tracker_mod, args.verbose)
        passed += ok
        events += n
        expiries += e
    print(f"tracker parity: {passed}/{len(scenarios)} scenarios match "
          f"({events} events, {expiries} expiries)")
    return 0 if passed == len(scenarios) else 1


if __name__ == "__main__":
    sys.exit(main())
