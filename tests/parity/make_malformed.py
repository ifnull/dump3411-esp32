#!/usr/bin/env python3
"""Write tests/parity/fixtures/malformed.txt: hand-built edge cases.

The encoder fixtures (gen_fixtures.c) only produce well-formed traffic. These
cover what a radio actually delivers too: truncation, bad lengths, placeholder
values, lookalike frames. Each is built here from bytes, not from either
decoder, so a fixture can't inherit a decoder's assumptions.

    python3 tests/parity/make_malformed.py
"""

import hashlib
import struct
from pathlib import Path

OUT = Path(__file__).parent / "fixtures" / "malformed.txt"

ASTM_OUI = bytes([0xFA, 0x0B, 0xBC])
NAN_OUI = bytes([0x50, 0x6F, 0x9A])
ODID_SERVICE_ID = hashlib.sha256(b"org.opendroneid.remoteid").digest()[:6]
TX_MAC = bytes([0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC])


def basic_id(uas: bytes, id_ua: int = 0x12) -> bytes:
    return bytes([0x02, id_ua]) + uas.ljust(20, b"\x00")[:20] + bytes(3)


def location(lat: float, lon: float, flags: int = 0x22) -> bytes:
    body = struct.pack("<BBBbii", flags, 90, 40, -3, round(lat * 1e7), round(lon * 1e7))
    alts = struct.pack("<HHH", 2200, 2250, 2090)
    return (bytes([0x12]) + body + alts).ljust(25, b"\x00")


def pack(msgs: list, size: int = 25, count: int | None = None) -> bytes:
    n = len(msgs) if count is None else count
    return bytes([0xF2, size, n]) + b"".join(msgs)


def mgmt(subtype: int, body: bytes, ftype: int = 0) -> bytes:
    fc0 = (subtype << 4) | (ftype << 2)
    hdr = bytes([fc0, 0x00, 0x00, 0x00]) + b"\xff" * 6 + TX_MAC + TX_MAC + b"\x00\x00"
    return hdr + body


def beacon(ies: bytes) -> bytes:
    fixed = bytes(8) + struct.pack("<HH", 100, 0x0401)
    return mgmt(8, fixed + ies)


def ie(tag: int, data: bytes) -> bytes:
    return bytes([tag, len(data)]) + data


def astm_ie(msg: bytes, counter: int = 3) -> bytes:
    return ie(221, ASTM_OUI + bytes([0x0D, counter]) + msg)


def nan_action(attrs: bytes, extra_subtype_byte: bool = False) -> bytes:
    head = bytes([0x04, 0x09]) + NAN_OUI + bytes([0x13])
    if extra_subtype_byte:
        head += bytes([0x00])
    return mgmt(13, head + attrs)


def sda(service_id: bytes, control: int, extras: bytes, service_info: bytes | None) -> bytes:
    body = service_id + bytes([0x01, 0x00, control]) + extras
    if service_info is not None:
        body += bytes([len(service_info)]) + service_info
    return bytes([0x03]) + struct.pack("<H", len(body)) + body


GOOD = basic_id(b"EDGE-CASE-0001")
LOC = location(30.25, -97.75)

CASES = [
    # -- single messages ------------------------------------------------------
    ("m", bytes([0x02]), "1 byte: too short to decode at all (null)"),
    ("m", GOOD[:24], "Basic ID one byte short: type only"),
    ("m", GOOD + b"\xAA\xBB", "Basic ID with trailing bytes past 25"),
    ("m", basic_id(b"AB\x00\x00CD"), "UAS ID with embedded NULs (only trailing ones strip)"),
    ("m", basic_id(b"\xffBAD\x80ID"), "UAS ID with non-ASCII bytes"),
    ("m", basic_id(b"X", id_ua=0x9F), "id_type 9 / ua_type 15: unknown enum names"),
    ("m", location(91.0, 10.0), "Location lat > 90: pre-GPS-lock placeholder"),
    ("m", location(10.0, -180.0000001), "Location lon just past -180"),
    ("m", location(-90.0, 180.0), "Location exactly on the lat/lon limits"),
    ("m", location(45.0, 45.0, flags=0x0F), "Location with every low flag bit set"),
    ("m", LOC[:24], "Location one byte short"),
    ("m", bytes([0x32, 0x00]) + b"Search and Rescue\x00\x00\x7f\x01\x00\x00", "Self ID with control bytes"),
    ("m", bytes([0x42, 0x02]) + struct.pack("<ii", 0x7FFFFFFF, 0) + bytes(15), "System with out-of-range operator lat"),
    ("m", bytes([0x42, 0x03]) + struct.pack("<ii", 0, 0) + bytes(9), "System at 19 bytes"),
    ("m", bytes([0x52, 0x00]) + b"OPERATOR".ljust(20, b"\x00"), "Operator ID at exactly 22 bytes"),
    ("m", bytes([0x52, 0x00]) + b"OPERATOR".ljust(19, b"\x00"), "Operator ID one byte short"),
    ("m", bytes([0x22]) + bytes(24), "Authentication: no decoder, type only"),
    ("m", bytes([0x72]) + bytes(24), "Unassigned message type 0x7"),
    # -- message packs --------------------------------------------------------
    ("m", bytes([0xF2, 25]), "Pack shorter than its 3-byte header"),
    ("m", pack([], count=0), "Pack header only, zero messages"),
    ("m", pack([GOOD, LOC]), "Two-message pack"),
    ("m", pack([GOOD, LOC], count=3), "Pack claims 3 messages, carries 2"),
    ("m", pack([GOOD, LOC[:24]]), "Pack whose second message is truncated"),
    ("m", pack([b"", b"", b""], size=0, count=3), "Pack with message size 0"),
    ("m", bytes([0xF2, 1, 3, 0x02, 0x12, 0x42]), "Pack with message size 1"),
    ("m", pack([GOOD[:10], LOC[:10]], size=10), "Pack with short message size 10"),
    ("m", pack([pack([GOOD], size=25)], size=28), "Pack nested inside a pack"),
    # -- beacon frames --------------------------------------------------------
    ("f", beacon(ie(0, b"RID") + astm_ie(pack([GOOD, LOC]))), "Beacon carrying a pack"),
    ("f", beacon(ie(221, bytes([0x00, 0x50, 0xF2, 0x02]) + bytes(4)) + astm_ie(GOOD)),
     "Beacon with another vendor IE before the ASTM one"),
    ("f", beacon(ie(221, ASTM_OUI + bytes([0x0D, 0x01]))), "ASTM vendor IE of length 5: too short"),
    ("f", beacon(astm_ie(GOOD)[:-4]), "ASTM vendor IE truncated by the frame end"),
    ("f", beacon(ie(221, ASTM_OUI + bytes([0x0E, 0x00]) + GOOD)), "ASTM OUI with wrong vendor type"),
    ("f", beacon(b""), "Beacon with no IEs"),
    ("f", mgmt(5, bytes(12) + astm_ie(GOOD)), "Probe response with an ASTM IE: ignored"),
    ("f", mgmt(8, astm_ie(GOOD), ftype=2), "Data frame: not management"),
    ("f", beacon(astm_ie(GOOD))[:23], "23-byte frame: shorter than a MAC header"),
    # -- NAN action frames ----------------------------------------------------
    ("f", nan_action(sda(ODID_SERVICE_ID, 0x10, b"", b"\x05" + pack([GOOD, LOC]))), "NAN SDA carrying a pack"),
    ("f", nan_action(sda(ODID_SERVICE_ID, 0x10, b"", b"\x05" + GOOD), extra_subtype_byte=True),
     "NAN with an extra OUI-subtype byte before the attributes"),
    ("f", nan_action(sda(ODID_SERVICE_ID, 0x10 | 0x40 | 0x04 | 0x08, b"\x00\x00" + b"\x02ab" + b"\x01c",
                         b"\x05" + GOOD)), "NAN SDA with binding bitmap and both filters"),
    ("f", nan_action(sda(bytes(6), 0x10, b"", b"\x05" + GOOD)), "NAN without the ODID service ID (AirDrop-like)"),
    ("f", nan_action(sda(ODID_SERVICE_ID, 0x00, b"", None)), "ODID service ID but no Service Info: unparsed"),
    ("f", nan_action(sda(ODID_SERVICE_ID, 0x10, b"", b"\x05")), "Service Info length 1: unparsed"),
    ("f", nan_action(sda(ODID_SERVICE_ID, 0x10, b"", b"\x05" + GOOD))[:-3], "SDA truncated by the frame end"),
    ("f", nan_action(sda(ODID_SERVICE_ID, 0x04, b"\x7f", None)), "Matching filter length runs past the SDA"),
]


def main() -> None:
    lines = [
        "# Generated by tests/parity/make_malformed.py. Do not edit by hand.",
        "",
    ]
    for mode, data, note in CASES:
        lines.append(f"{mode} {data.hex().upper()}  # {note}")
    OUT.write_text("\n".join(lines) + "\n")
    print(f"wrote {len(CASES)} cases to {OUT}")


if __name__ == "__main__":
    main()
