#!/usr/bin/env python3
"""Check the C decoder against dump3411's Python decoder, and both against
the reference encoder's input values.

Two checks run on every fixture:

1. Parity: the C output (host/odid_dump) must match dump3411's output for the
   same bytes, field for field. Same keys present, same strings and integers,
   and floats within one wire LSB (tolerances below).
2. Ground truth: wherever a decoded message's bytes appear in
   fixtures/encoder_expected.jsonl, each decoder must reproduce the values the
   reference encoder was given. Parity alone can't catch a bug the C port
   copied faithfully from dump3411; this can.

    cmake -S host -B build/host && cmake --build build/host
    python3 tests/parity/run_parity.py [--dump3411 PATH]

dump3411 is imported from a checkout (default: ../dump3411 next to this repo,
or $DUMP3411_DIR). Exit status is non-zero on any failure.
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).parent / "fixtures"
ODID_DUMP = ROOT / "build" / "host" / "odid_dump"

# One wire LSB per field, the most the C and Python values may differ by
# (field-level semantic parity; see the 2026-08-14 design doc). Encodings are
# in dump3411's wifi_feeder.py parse_location / parse_system_msg.
TOLERANCE = {
    "latitude": 1e-7,          # int32 * 1e-7 deg
    "longitude": 1e-7,
    "operator_lat": 1e-7,
    "operator_lon": 1e-7,
    "altitude_geo": 0.5,       # uint16 * 0.5 - 1000 m
    "height_agl": 0.5,
    "alt_takeoff_geo": 0.5,
    "vertical_speed": 0.5,     # int8 * 0.5 m/s
    "heading": 1.0,            # uint8 deg (+180 for the E/W half)
    "ground_speed": 0.75,      # uint8 * 0.25, or * 0.75 above 63.75 m/s
}
EXACT = {
    "message_type", "raw_hex", "messages",
    "id_type", "ua_type", "id_type_raw", "ua_type_raw", "uas_id",
    "height_type", "description_type", "description",
    "area_count", "area_radius_m", "operator_location_type",
    "operator_id_type", "operator_id",
    # frame-level keys
    "mgmt", "subtype", "addr2", "transport", "nan_unparsed", "decoded",
}
EPS = 1e-9


def load_fixtures():
    cases = []
    for path in sorted(FIXTURES.glob("*.txt")):
        for lineno, raw in enumerate(path.read_text().splitlines(), 1):
            text, _, comment = raw.partition("#")
            text = text.strip()
            if not text:
                continue
            mode, hexstr = text.split(None, 1)
            cases.append({
                "where": f"{path.name}:{lineno}",
                "note": comment.strip(),
                "mode": mode,
                "data": bytes.fromhex(hexstr.replace(" ", "")),
            })
    return cases


def load_expected():
    expected = {}
    path = FIXTURES / "encoder_expected.jsonl"
    if path.exists():
        for line in path.read_text().splitlines():
            if line.strip():
                row = json.loads(line)
                expected[row["hex"]] = row["expect"]
    return expected


class Oracle:
    """dump3411's decode path, with the radio and tracker left out."""

    def __init__(self, dump3411_dir: Path):
        sys.path.insert(0, str(dump3411_dir))
        import logging
        import wifi_feeder  # noqa: E402  (stdlib-only imports)
        logging.getLogger().setLevel(logging.WARNING)
        self.wf = wifi_feeder

    def message(self, data: bytes):
        return self.wf.decode_rid_message(data)

    def frame(self, data: bytes):
        # Mirrors WiFiFeeder._on_packet, minus radiotap (none on ESP32).
        header = self.wf._parse_dot11_mgmt(data)
        if header is None:
            return {"mgmt": False}
        subtype, addr2, body_offset = header
        body = data[body_offset:]
        out = {"mgmt": True, "subtype": subtype, "addr2": addr2,
               "transport": None, "nan_unparsed": False, "decoded": None}
        payload = None
        if subtype == 8:
            payload = self.wf._extract_beacon_rid(body)
            if payload is not None:
                out["transport"] = "beacon"
        elif subtype == 13 and self.wf._is_nan_action(body):
            payload = self.wf._extract_nan_odid(body)
            if payload is None:
                out["nan_unparsed"] = True
            else:
                out["transport"] = "nan"
        if payload is not None:
            out["decoded"] = self.wf.decode_rid_message(payload)
        return out


def compare(c, py, path, diffs, worst):
    """Append a description of every C/Python difference to diffs."""
    if isinstance(c, dict) and isinstance(py, dict):
        for key in sorted(set(c) | set(py)):
            where = f"{path}.{key}"
            if key not in TOLERANCE and key not in EXACT:
                diffs.append(f"{where}: unrecognized field, no comparison rule")
            elif key not in c:
                diffs.append(f"{where}: only Python has it ({py[key]!r})")
            elif key not in py:
                diffs.append(f"{where}: only C has it ({c[key]!r})")
            elif key in TOLERANCE:
                delta = abs(c[key] - py[key])
                worst[key] = max(worst.get(key, 0.0), delta)
                if delta > TOLERANCE[key] + EPS:
                    diffs.append(f"{where}: C {c[key]} vs Python {py[key]} (tolerance {TOLERANCE[key]})")
            else:
                compare(c[key], py[key], where, diffs, worst)
    elif isinstance(c, list) and isinstance(py, list):
        if len(c) != len(py):
            diffs.append(f"{path}: C has {len(c)} items, Python {len(py)}")
        for i, (a, b) in enumerate(zip(c, py)):
            compare(a, b, f"{path}[{i}]", diffs, worst)
    elif c != py:
        diffs.append(f"{path}: C {c!r} vs Python {py!r}")


def check_truth(decoded, expected, label, path, errors, checked):
    """Walk a decoded tree; check every message the encoder has values for."""
    if isinstance(decoded, list):
        for i, item in enumerate(decoded):
            check_truth(item, expected, label, f"{path}[{i}]", errors, checked)
        return
    if not isinstance(decoded, dict):
        return
    if "decoded" in decoded:
        check_truth(decoded["decoded"], expected, label, path, errors, checked)
        return
    want = expected.get(decoded.get("raw_hex"))
    if want is not None:
        checked.add((label, decoded["raw_hex"]))
        for key, value in want.items():
            got = decoded.get(key)
            if value is None:
                if key in decoded:
                    errors.append(f"{label} {path}.{key}: expected absent, got {got!r}")
            elif got is None:
                errors.append(f"{label} {path}.{key}: missing, expected {value!r}")
            elif key in TOLERANCE:
                if abs(got - value) > TOLERANCE[key] / 2 + EPS:
                    errors.append(f"{label} {path}.{key}: got {got}, encoder had {value}")
            elif got != value:
                errors.append(f"{label} {path}.{key}: got {got!r}, encoder had {value!r}")
    check_truth(decoded.get("messages", []), expected, label, path, errors, checked)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    default = os.environ.get("DUMP3411_DIR", str(ROOT.parent / "dump3411"))
    parser.add_argument("--dump3411", default=default, help="dump3411 checkout (default: %(default)s)")
    parser.add_argument("-v", "--verbose", action="store_true", help="print every case")
    args = parser.parse_args()

    dump3411_dir = Path(args.dump3411).resolve()
    if not (dump3411_dir / "wifi_feeder.py").exists():
        print(f"dump3411 not found at {dump3411_dir} (use --dump3411 or DUMP3411_DIR)")
        return 2
    if not ODID_DUMP.exists():
        print(f"{ODID_DUMP} missing; build it: cmake -S host -B build/host && cmake --build build/host")
        return 2

    oracle = Oracle(dump3411_dir)
    cases = load_fixtures()
    expected = load_expected()

    stdin = "".join(f"{c['mode']} {c['data'].hex()}\n" for c in cases)
    run = subprocess.run([str(ODID_DUMP)], input=stdin, capture_output=True, text=True)
    if run.returncode != 0:
        print(f"odid_dump failed (exit {run.returncode}):\n{run.stderr}")
        return 1
    c_results = [json.loads(line) for line in run.stdout.splitlines()]
    if len(c_results) != len(cases):
        print(f"odid_dump returned {len(c_results)} results for {len(cases)} fixtures")
        return 1

    rev = subprocess.run(["git", "-C", str(dump3411_dir), "describe", "--always", "--dirty"],
                         capture_output=True, text=True).stdout.strip() or "unknown"
    print(f"dump3411 oracle: {dump3411_dir} @ {rev}")

    parity_fail = 0
    truth_errors = []
    checked = set()
    worst = {}
    for case, c_out in zip(cases, c_results):
        py_out = oracle.message(case["data"]) if case["mode"] == "m" else oracle.frame(case["data"])
        diffs = []
        compare(c_out, py_out, "$", diffs, worst)
        if diffs:
            parity_fail += 1
            print(f"PARITY FAIL {case['where']}  {case['note']}")
            for d in diffs:
                print(f"    {d}")
        elif args.verbose:
            print(f"ok   {case['where']}  {case['note']}")
        for label, out in (("C", c_out), ("Python", py_out)):
            errs = []
            check_truth(out, expected, label, "$", errs, checked)
            truth_errors.extend(f"{case['where']} {e}" for e in errs)

    print()
    print(f"parity: {len(cases) - parity_fail}/{len(cases)} fixtures match")
    if worst:
        print("largest C-vs-Python float difference per field:")
        for key in sorted(worst):
            print(f"    {key:16} {worst[key]:.3g}  (tolerance {TOLERANCE[key]})")

    for label in ("C", "Python"):
        seen = {h for lab, h in checked if lab == label}
        missing = set(expected) - seen
        if missing:
            truth_errors.append(f"{label}: {len(missing)} encoder messages never decoded")
    print(f"ground truth: {len(expected)} encoder messages, {len(truth_errors)} problems")
    for e in truth_errors:
        print(f"    {e}")

    return 1 if parity_fail or truth_errors else 0


if __name__ == "__main__":
    sys.exit(main())
