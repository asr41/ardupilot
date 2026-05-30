"""
iridium_decoder.py

Ground-station codec for the iridium_telemetry.lua binary protocol.

  decode_mo(hex_str)  → dict   Decode a vehicle telemetry packet (MO)
  encode_*(...)       → str    Encode a ground command (MT) as a hex string

The hex strings are raw payload bytes with no SBD framing.
Use sbd_checksum(payload_bytes) when your networking layer needs to wrap
the payload for AT+SBDWB transmission.
"""

import struct

# ── Magic / type constants ────────────────────────────────────────────────────

MO_MAGIC = b'\xAD\x12'
MT_MAGIC = b'\xAD\x34'

MO_TYPE_TELEMETRY = 0x01

CMD_WAYPOINT  = 0x01  # 15 bytes total
CMD_PARAMETER = 0x02  # 23 bytes total
CMD_MODE      = 0x03  #  4 bytes total
CMD_ARM       = 0x04  #  4 bytes total
CMD_TX_RATE   = 0x05  #  5 bytes total

# ── ArduPlane flight mode names ───────────────────────────────────────────────

PLANE_MODES = {
    0:  "MANUAL",
    1:  "CIRCLE",
    2:  "STABILIZE",
    3:  "TRAINING",
    4:  "ACRO",
    5:  "FBWA",
    6:  "FBWB",
    7:  "CRUISE",
    8:  "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    13: "TAKEOFF",
    14: "AVOID_ADSB",
    15: "GUIDED",
    17: "QSTABILIZE",
    18: "QHOVER",
    19: "QLOITER",
    20: "QLAND",
    21: "QRTL",
    22: "QAUTOTUNE",
    23: "QACRO",
    24: "THERMAL",
    25: "LOITER_ALT_QLAND",
}

GPS_FIX_NAMES = {
    0: "No GPS",
    1: "No Fix",
    2: "2D Fix",
    3: "3D Fix",
    4: "DGPS",
    5: "RTK",
}

# ── MO packet struct ──────────────────────────────────────────────────────────
# magic(2s) type(B) seq(H) lat(i) lng(i) alt(i)
# vn(h) ve(h) vd(h) roll(h) pitch(h) yaw(H)
# bat_mv(H) bat_ca(h) sats(B) fix(B) mode(B) armed(B)
_MO_FMT  = ">2sBHiiihhhhhhHhBBBB"
_MO_SIZE = struct.calcsize(_MO_FMT)   # 37

# ── Iridium SBD checksum ──────────────────────────────────────────────────────

def sbd_checksum(payload: bytes) -> bytes:
    """Two-byte big-endian sum-mod-65536 checksum for AT+SBDWB."""
    total = sum(payload) & 0xFFFF
    return struct.pack(">H", total)


# ── MO decoder ────────────────────────────────────────────────────────────────

class IridiumDecodeError(ValueError):
    pass


def decode_mo(hex_str: str) -> dict:
    """
    Decode a 37-byte MO telemetry packet from a hex string.

    Returns a dict with human-readable fields and their raw values where useful:
      lat_deg, lng_deg, alt_m          – position
      vel_n_ms, vel_e_ms, vel_d_ms    – velocity (m/s)
      roll_deg, pitch_deg, yaw_deg    – attitude
      bat_v, bat_a                    – battery
      gps_sats, gps_fix, gps_fix_name
      mode, mode_name
      armed, seq
    """
    try:
        raw = bytes.fromhex(hex_str.strip().replace(" ", ""))
    except ValueError as e:
        raise IridiumDecodeError(f"Invalid hex string: {e}") from e

    if len(raw) != _MO_SIZE:
        raise IridiumDecodeError(
            f"Expected {_MO_SIZE} bytes, got {len(raw)}"
        )

    fields = struct.unpack(_MO_FMT, raw)
    (magic, pkt_type, seq,
     lat_e7, lng_e7, alt_cm,
     vn_cms, ve_cms, vd_cms,
     roll_cd, pitch_cd, yaw_cd,
     bat_mv, bat_ca,
     sats, fix, mode, armed) = fields

    if magic != MO_MAGIC:
        raise IridiumDecodeError(
            f"Bad magic: expected AD12, got {magic.hex().upper()}"
        )
    if pkt_type != MO_TYPE_TELEMETRY:
        raise IridiumDecodeError(f"Unknown packet type: 0x{pkt_type:02X}")

    return {
        "seq":          seq,
        "lat_deg":      lat_e7  / 1e7,
        "lng_deg":      lng_e7  / 1e7,
        "alt_m":        alt_cm  / 100.0,
        "vel_n_ms":     vn_cms  / 100.0,
        "vel_e_ms":     ve_cms  / 100.0,
        "vel_d_ms":     vd_cms  / 100.0,
        "roll_deg":     roll_cd  / 100.0,
        "pitch_deg":    pitch_cd / 100.0,
        "yaw_deg":      yaw_cd   / 100.0,
        "bat_v":        bat_mv  / 1000.0,
        "bat_a":        bat_ca  / 100.0,
        "gps_sats":     sats,
        "gps_fix":      fix,
        "gps_fix_name": GPS_FIX_NAMES.get(fix, f"Unknown({fix})"),
        "mode":         mode,
        "mode_name":    PLANE_MODES.get(mode, f"Mode({mode})"),
        "armed":        bool(armed),
    }


# ── MT encoders ───────────────────────────────────────────────────────────────

def _mt_header(cmd: int) -> bytes:
    return MT_MAGIC + bytes([cmd])


def encode_waypoint(lat_deg: float, lng_deg: float, alt_m: float) -> str:
    """
    Encode a Waypoint command (0x01).
    lat/lng in decimal degrees, alt in metres (MSL).
    Returns a 15-byte hex string.
    """
    lat_e7 = int(round(lat_deg * 1e7))
    lng_e7 = int(round(lng_deg * 1e7))
    alt_cm = int(round(alt_m * 100))
    payload = _mt_header(CMD_WAYPOINT) + struct.pack(">iii", lat_e7, lng_e7, alt_cm)
    return payload.hex()


def encode_parameter(name: str, value: float) -> str:
    """
    Encode a Parameter command (0x02).
    name is the ArduPilot param name (up to 16 chars).
    Returns a 23-byte hex string.
    """
    name_bytes = name.encode("ascii", errors="replace")[:16].ljust(16, b'\x00')
    payload = _mt_header(CMD_PARAMETER) + name_bytes + struct.pack(">f", value)
    return payload.hex()


def encode_mode(mode: int) -> str:
    """
    Encode a Mode command (0x03).
    mode is the ArduPlane flight mode number (see PLANE_MODES).
    Returns a 4-byte hex string.
    """
    payload = _mt_header(CMD_MODE) + struct.pack(">B", mode)
    return payload.hex()


def encode_arm(arm: bool) -> str:
    """
    Encode an Arm/Disarm command (0x04).
    arm=True to arm, arm=False to disarm.
    Returns a 4-byte hex string.
    """
    payload = _mt_header(CMD_ARM) + struct.pack(">B", 1 if arm else 0)
    return payload.hex()


def encode_tx_rate(rate_seconds: int) -> str:
    """
    Encode a TX Rate command (0x05).
    rate_seconds is clamped to [10, 3600].
    Returns a 5-byte hex string.
    """
    rate = max(10, min(3600, int(rate_seconds)))
    payload = _mt_header(CMD_TX_RATE) + struct.pack(">H", rate)
    return payload.hex()


# ── Quick CLI for testing ─────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    import json

    if len(sys.argv) < 2:
        print("Usage:")
        print("  python iridium_decoder.py decode <hex>")
        print("  python iridium_decoder.py waypoint <lat> <lng> <alt_m>")
        print("  python iridium_decoder.py parameter <name> <value>")
        print("  python iridium_decoder.py mode <mode_num>")
        print("  python iridium_decoder.py arm <1|0>")
        print("  python iridium_decoder.py txrate <seconds>")
        sys.exit(1)

    cmd = sys.argv[1].lower()

    if cmd == "decode":
        result = decode_mo(sys.argv[2])
        print(json.dumps(result, indent=2))

    elif cmd == "waypoint":
        lat, lng, alt = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
        print(encode_waypoint(lat, lng, alt))

    elif cmd == "parameter":
        print(encode_parameter(sys.argv[2], float(sys.argv[3])))

    elif cmd == "mode":
        mode_num = int(sys.argv[2])
        print(f"Encoding mode {mode_num} ({PLANE_MODES.get(mode_num, 'unknown')})")
        print(encode_mode(mode_num))

    elif cmd == "arm":
        print(encode_arm(sys.argv[2] == "1"))

    elif cmd == "txrate":
        print(encode_tx_rate(int(sys.argv[2])))

    else:
        print(f"Unknown command: {cmd}", file=sys.stderr)
        sys.exit(1)
