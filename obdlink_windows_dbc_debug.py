import argparse
import csv
import re
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

import cantools
import serial


# ============================================================
# ELM / OBDLink helpers
# ============================================================

def send_at(ser, cmd, delay=0.15, verbose=True, timeout=3.0):
    if verbose:
        print(f">>> {cmd}")
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    ser.write((cmd + "\r").encode("ascii", errors="ignore"))
    ser.flush()
    t0 = time.perf_counter()
    buf = b""
    while time.perf_counter() - t0 < timeout:
        n = ser.in_waiting
        chunk = ser.read(n if n > 0 else 1)
        if chunk:
            buf += chunk
            if b">" in buf:
                break
        else:
            time.sleep(0.002)
    text = buf.decode("ascii", errors="ignore").replace("\r", "\n")
    if verbose:
        for line in [x.strip() for x in text.split("\n") if x.strip()]:
            if line != cmd:
                print(line)
    time.sleep(delay)
    return text


def restart_monitor(ser, stats=None, reason=""):
    """
    Honda-style instant restart:
      1. space + CR  -> stops ATMA
      2. 30 ms settle
      3. reset_input_buffer  -> discard stale bytes
      4. ATMA  -> resume
    """
    if stats is not None:
        stats["atma_restarts"] = stats.get("atma_restarts", 0) + 1
        stats["last_restart_reason"] = reason
    print(f"[WARN] Restart ATMA  reason={reason!r}")
    try:
        ser.write(b" \r")
        ser.flush()
        time.sleep(0.03)
        ser.reset_input_buffer()
        ser.write(b"ATMA\r")
        ser.flush()
    except Exception as e:
        print(f"[ERROR] Restart failed: {e}")


OBDLINK_BAUD_CODES = {230400: "34", 460800: "1A", 921600: "0D"}


def try_upgrade_baud(ser, target_baud):
    if target_baud not in OBDLINK_BAUD_CODES:
        print(f"[WARN] Baud {target_baud} not in supported list.")
        return ser, ser.baudrate
    brd_code = OBDLINK_BAUD_CODES[target_baud]
    port = ser.port
    original_baud = ser.baudrate
    print(f"[INFO] Baud upgrade {original_baud} -> {target_baud} (AT BRD {brd_code})")
    try:
        ser.reset_input_buffer()
        ser.write(f"AT BRD {brd_code}\r".encode("ascii", errors="ignore"))
        ser.flush()
        t0 = time.perf_counter()
        buf = b""
        while time.perf_counter() - t0 < 0.4:
            n = ser.in_waiting
            if n > 0:
                buf += ser.read(n)
                if b"OK" in buf or b"?" in buf:
                    break
            time.sleep(0.003)
        if b"OK" not in buf:
            print(f"[WARN] AT BRD rejected: {buf!r}")
            return ser, original_baud
        ser.close()
        time.sleep(0.02)
        new_ser = serial.Serial(port, target_baud, timeout=0.01, write_timeout=0.5, rtscts=True)

        def wait_prompt(t_s):
            t_start = time.perf_counter()
            bl = b""
            while time.perf_counter() - t_start < t_s:
                n = new_ser.in_waiting
                if n > 0:
                    bl += new_ser.read(n)
                    if b">" in bl or b"OK" in bl:
                        return True, bl
                time.sleep(0.003)
            return False, bl

        for _ in range(3):
            new_ser.write(b"\r")
            new_ser.flush()
            ok, _ = wait_prompt(0.6)
            if ok:
                print(f"[INFO] Baud upgrade OK -> {target_baud}.")
                return new_ser, target_baud
        new_ser.write(b"AT\r")
        new_ser.flush()
        ok, _ = wait_prompt(1.2)
        if ok:
            print(f"[INFO] Baud upgrade OK -> {target_baud}.")
            return new_ser, target_baud
        print("[WARN] Baud upgrade confirm failed. Rolling back.")
        new_ser.close()
        time.sleep(0.1)
        recovered = serial.Serial(port, original_baud, timeout=0.01, write_timeout=0.2, rtscts=True)
        send_at(recovered, "ATZ", delay=0.5, verbose=False)
        return recovered, original_baud
    except Exception as e:
        print(f"[WARN] Baud upgrade exception: {e}")
        try:
            return serial.Serial(port, original_baud, timeout=0.01, write_timeout=0.2, rtscts=True), original_baud
        except Exception:
            return ser, original_baud


# ============================================================
# CAN line parser  (kept as-is – more robust than Honda's)
# ============================================================

HEX_PAIR_RE = re.compile(r"^[0-9A-Fa-f]{2}$")
HEX_ID_RE   = re.compile(r"^[0-9A-Fa-f]{1,8}$")


def parse_atma_line(line):
    if not line:
        return None
    s = line.strip()
    if not s:
        return None
    if s == ">":
        return ("__PROMPT__", b"")
    s = s.replace(">", " ").strip()
    if not s:
        return None
    upper = s.upper()
    for kw in ["NO DATA", "STOPPED", "BUFFER FULL", "CAN ERROR", "BUS ERROR", "UNABLE TO CONNECT"]:
        if kw in upper:
            return ("__ADAPTER_STATUS__", upper.encode("ascii", errors="ignore"))
    if upper in ["OK", "?"]:
        return None
    if "SEARCHING" in upper:
        return None
    raw_tokens = re.split(r"[\s,]+", s)
    tokens = []
    for tok in raw_tokens:
        tok = tok.strip().replace(":", "")
        if not tok:
            continue
        if not re.fullmatch(r"[0-9A-Fa-f]+", tok):
            continue
        tokens.append(tok)
    if len(tokens) == 1:
        lone = tokens[0]
        if len(lone) >= 5:
            if len(lone) >= 10 and (len(lone) - 8) % 2 == 0:
                tokens = [lone[:8]] + [lone[i:i + 2] for i in range(8, len(lone), 2)]
            elif (len(lone) - 3) % 2 == 0:
                tokens = [lone[:3]] + [lone[i:i + 2] for i in range(3, len(lone), 2)]
    if len(tokens) < 2:
        return None
    can_id_token = tokens[0]
    if not HEX_ID_RE.match(can_id_token):
        return None
    try:
        can_id = int(can_id_token, 16)
    except Exception:
        return None
    data_tokens = tokens[1:]
    expanded = []
    for tok in data_tokens:
        if len(tok) > 2 and len(tok) % 2 == 0:
            expanded.extend([tok[i:i + 2] for i in range(0, len(tok), 2)])
        else:
            expanded.append(tok)
    data_tokens = expanded
    if len(data_tokens) >= 2:
        try:
            maybe_dlc = int(data_tokens[0], 16)
            remaining = data_tokens[1:]
            if (0 <= maybe_dlc <= 8
                    and len(remaining) == maybe_dlc
                    and all(HEX_PAIR_RE.match(x) for x in remaining)):
                data_tokens = remaining
        except Exception:
            pass
    data_bytes = []
    for tok in data_tokens:
        if len(tok) != 2 or not HEX_PAIR_RE.match(tok):
            continue
        data_bytes.append(int(tok, 16))
    if not data_bytes:
        return None
    return can_id, bytes(data_bytes[:8])


# ============================================================
# Signal presets
# ============================================================

PRESETS = {
    "steer_rpm":         ("150", "7F0"),
    "steer":             ("156", "7FF"),
    "rpm":               ("158", "7FF"),
    "wheel":             ("1D0", "7FF"),
    "rough_wheel":       ("255", "7FF"),
    "gas":               ("130", "7FF"),
    "gas2":              ("13C", "7FF"),
    "brake":             ("17C", "7FF"),
    "brake_pressure":    ("1E7", "7FF"),
    "vsa":               ("1A4", "7FF"),
    "car_speed":         ("309", "7FF"),
    "gear_191":          ("191", "7FF"),
    "speed_related_100": ("100", "700"),
    "speed_related_200": ("200", "700"),
    "speed_related_300": ("300", "700"),
    "range_000":         ("000", "700"),
    "range_100":         ("100", "700"),
    "range_200":         ("200", "700"),
    "range_300":         ("300", "700"),
    "range_400":         ("400", "700"),
}


# ============================================================
# Watch signals for live dashboard
# ============================================================

WATCH_SIGNALS = [
    ("VEHICLE_SPEED", [
        "CAR_SPEED.ROUGH_CAR_SPEED_3", "ROUGH_CAR_SPEED_3",
        "CAR_SPEED.CAR_SPEED", "CAR_SPEED",
    ]),
    ("BRAKE_ACTIVE", [
        "POWERTRAIN_DATA.BRAKE_SWITCH", "BRAKE_SWITCH",
        "POWERTRAIN_DATA.BRAKE_PRESSED", "BRAKE_PRESSED",
        "VSA_STATUS.USER_BRAKE", "USER_BRAKE",
    ]),
    ("STEER_ANGLE", [
        "STEERING_SENSORS.STEER_ANGLE", "STEER_ANGLE",
    ]),
    ("WHEEL_SPEED_FL", [
        "WHEEL_SPEEDS.WHEEL_SPEED_FL", "ROUGH_WHEEL_SPEED.WHEEL_SPEED_FL", "WHEEL_SPEED_FL",
    ]),
    ("WHEEL_SPEED_FR", [
        "WHEEL_SPEEDS.WHEEL_SPEED_FR", "ROUGH_WHEEL_SPEED.WHEEL_SPEED_FR", "WHEEL_SPEED_FR",
    ]),
    ("WHEEL_SPEED_RL", [
        "WHEEL_SPEEDS.WHEEL_SPEED_RL", "ROUGH_WHEEL_SPEED.WHEEL_SPEED_RL", "WHEEL_SPEED_RL",
    ]),
    ("WHEEL_SPEED_RR", [
        "WHEEL_SPEEDS.WHEEL_SPEED_RR", "ROUGH_WHEEL_SPEED.WHEEL_SPEED_RR", "WHEEL_SPEED_RR",
    ]),
]


# ============================================================
# Display helpers
# ============================================================

def fmt_value(v):
    if v is None:
        return "NA"
    if isinstance(v, float):
        return f"{v:.4f}"
    return str(v)


def get_first_available(last_values, names):
    for name in names:
        if name in last_values:
            return last_values[name]
    return None


def get_first_time(signal_time, names):
    for name in names:
        if name in signal_time:
            return signal_time[name]
    return None


def get_value_and_age_ms(last_values, signal_time, aliases, now_perf):
    v = get_first_available(last_values, aliases)
    t = get_first_time(signal_time, aliases)
    if v is None or t is None:
        return None, None
    return v, (now_perf - t) * 1000.0


def format_csv_value(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return int(v)
    return v


def build_all_signal_aliases(db):
    aliases = {}
    for msg in db.messages:
        for sig in msg.signals:
            full = f"{msg.name}.{sig.name}"
            aliases[full] = [full, sig.name]
    return aliases


def print_dashboard(
    elapsed, last_values, signal_time,
    id_counts, accepted_counts, throttled_counts,
    decode_ok, decode_fail,
    now_perf, stats, processed_lines,
    stale_ms, show_all_signals, last_line_time,
):
    print("-" * 110)
    print(
        f"t={elapsed:.2f}s | "
        f"raw={sum(id_counts.values())} | "
        f"accepted={sum(accepted_counts.values())} | "
        f"throttled={sum(throttled_counts.values())} | "
        f"dec_ok={sum(decode_ok.values())} | "
        f"dec_fail={sum(decode_fail.values())} | "
        f"lines={processed_lines} | "
        f"bytes={stats.get('serial_bytes', 0)} | "
        f"restarts={stats.get('atma_restarts', 0)}"
    )
    if last_line_time is None:
        print("Serial stream        : NO LINE YET")
    else:
        age_ms = (now_perf - last_line_time) * 1000.0
        state  = "STALE" if age_ms > stale_ms else "LIVE"
        print(f"Serial stream        : {state}, age={age_ms:.1f} ms")
    for label, aliases in WATCH_SIGNALS:
        v, age_ms = get_value_and_age_ms(last_values, signal_time, aliases, now_perf)
        if age_ms is None:
            print(f"{label:<20}: {fmt_value(v):<14} age=NA          STALE")
        else:
            state = "STALE" if age_ms > stale_ms else "LIVE"
            print(f"{label:<20}: {fmt_value(v):<14} age={age_ms:8.1f} ms  {state}")
    if id_counts:
        print(f"Top raw IDs          : {' '.join(f'0x{k:X}:{v}' for k, v in id_counts.most_common(10))}")
    if accepted_counts:
        print(f"Top accepted IDs     : {' '.join(f'0x{k:X}:{v}' for k, v in accepted_counts.most_common(10))}")
    if throttled_counts:
        print(f"Top throttled IDs    : {' '.join(f'0x{k:X}:{v}' for k, v in throttled_counts.most_common(10))}")
    if decode_fail:
        print(f"Decode fail IDs      : {' '.join(f'0x{k:X}:{v}' for k, v in decode_fail.most_common(5))}")
    if stats.get("last_restart_reason"):
        print(f"Last restart reason  : {stats['last_restart_reason']}")
    if show_all_signals:
        full_keys = sorted(k for k in last_values if "." in k)
        if full_keys:
            print("All decoded signals  :")
            for key in full_keys:
                print(f"  {key}={fmt_value(last_values.get(key))}")
    print()


# ============================================================
# Snapshot CSV helpers
# ============================================================

def make_snapshot_header():
    h = ["time_wall", "time_perf", "rel_time"]
    for label, _ in WATCH_SIGNALS:
        h += [label, label + "_age_ms"]
    return h


def make_snapshot_row(now_wall, now_perf, rel_time, last_values, signal_time):
    row = [f"{now_wall:.6f}", f"{now_perf:.6f}", f"{rel_time:.6f}"]
    for _, aliases in WATCH_SIGNALS:
        v, age_ms = get_value_and_age_ms(last_values, signal_time, aliases, now_perf)
        row.append(format_csv_value(v))
        row.append("" if age_ms is None else f"{age_ms:.3f}")
    return row


# ============================================================
# Selected signal CSV helpers
# ============================================================

SELECTED_ALIASES = {
    "vehicle_speed": [
        "CAR_SPEED.ROUGH_CAR_SPEED_3", "ROUGH_CAR_SPEED_3",
        "CAR_SPEED.CAR_SPEED", "CAR_SPEED",
    ],
    "gear": [
        "GEARBOX_CVT.GEAR_SHIFTER", "GEARBOX_AUTO.GEAR_SHIFTER",
        "GEARBOX.GEAR_SHIFTER", "GEAR_SHIFTER",
    ],
    "gas_pedal": [
        "GAS_PEDAL_2.CAR_GAS", "GAS_PEDAL.CAR_GAS", "CAR_GAS",
        "POWERTRAIN_DATA.PEDAL_GAS", "PEDAL_GAS",
    ],
    "brake_active": [
        "POWERTRAIN_DATA.BRAKE_SWITCH", "BRAKE_SWITCH",
        "POWERTRAIN_DATA.BRAKE_PRESSED", "BRAKE_PRESSED",
        "VSA_STATUS.USER_BRAKE", "USER_BRAKE",
    ],
    "steer_angle": [
        "STEERING_SENSORS.STEER_ANGLE", "STEER_ANGLE",
    ],
    "wheel_speed_fl": [
        "WHEEL_SPEEDS.WHEEL_SPEED_FL", "ROUGH_WHEEL_SPEED.WHEEL_SPEED_FL", "WHEEL_SPEED_FL",
    ],
    "wheel_speed_fr": [
        "WHEEL_SPEEDS.WHEEL_SPEED_FR", "ROUGH_WHEEL_SPEED.WHEEL_SPEED_FR", "WHEEL_SPEED_FR",
    ],
    "wheel_speed_rl": [
        "WHEEL_SPEEDS.WHEEL_SPEED_RL", "ROUGH_WHEEL_SPEED.WHEEL_SPEED_RL", "WHEEL_SPEED_RL",
    ],
    "wheel_speed_rr": [
        "WHEEL_SPEEDS.WHEEL_SPEED_RR", "ROUGH_WHEEL_SPEED.WHEEL_SPEED_RR", "WHEEL_SPEED_RR",
    ],
}


def convert_selected_value(name, value, speed_unit):
    if value is None:
        return None
    if name in {"vehicle_speed", "wheel_speed_fl", "wheel_speed_fr", "wheel_speed_rl", "wheel_speed_rr"}:
        try:
            v = float(value)
            if speed_unit == "kph_to_mps":
                return v / 3.6
            if speed_unit == "mps_to_kph":
                return v * 3.6
            return v
        except Exception:
            return value
    if name == "brake_active":
        try:
            return int(value) != 0
        except Exception:
            return bool(value)
    return value


def make_selected_header():
    h = ["time_wall", "time_perf", "rel_time", "trigger_can_id"]
    for name in SELECTED_ALIASES:
        h += [name, name + "_age_ms"]
    return h


def make_selected_row(now_wall, now_perf, rel_t, trigger_can_id,
                      last_values, signal_time, speed_unit):
    row = [
        f"{now_wall:.6f}", f"{now_perf:.6f}",
        f"{rel_t:.6f}", f"0x{trigger_can_id:X}",
    ]
    for name, aliases in SELECTED_ALIASES.items():
        v, age_ms = get_value_and_age_ms(last_values, signal_time, aliases, now_perf)
        v = convert_selected_value(name, v, speed_unit)
        row.append(format_csv_value(v))
        row.append("" if age_ms is None else f"{age_ms:.3f}")
    return row


# ============================================================
# Per-ID rate limiter
# ============================================================

def should_accept_id(can_id, now_perf, last_accept_time_by_id, max_id_hz):
    if max_id_hz is None or max_id_hz <= 0:
        last_accept_time_by_id[can_id] = now_perf
        return True
    min_dt = 1.0 / max_id_hz
    last_t = last_accept_time_by_id.get(can_id)
    if last_t is None or (now_perf - last_t) >= min_dt:
        last_accept_time_by_id[can_id] = now_perf
        return True
    return False


def list_signals_and_exit(db):
    keywords = [
        "SPEED", "WHEEL", "BRAKE", "BLINKER", "TURN", "SIGNAL",
        "LAMP", "LIGHT", "LEFT", "RIGHT", "STALK", "STEER",
        "GAS", "PEDAL", "RPM", "XMISSION", "GEAR",
    ]
    print("\n[INFO] Signals matched by keywords:")
    for msg in db.messages:
        for sig in msg.signals:
            if any(k in sig.name.upper() or k in msg.name.upper() for k in keywords):
                print(
                    f"  ID=0x{msg.frame_id:<4X}  "
                    f"MSG={msg.name:<35}  "
                    f"SIG={sig.name:<35}  "
                    f"LEN={msg.length}"
                )
    sys.exit(0)


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Single-threaded CAN logger for Honda (OBDLink EX)")
    parser.add_argument("--port",         required=False, help="Serial port, e.g. COM10")
    parser.add_argument("--dbc",          required=True,  help="DBC file path")
    parser.add_argument("--duration",     type=float, default=0.0,
                        help="Seconds to run. 0 = infinite until Ctrl+C.")
    parser.add_argument("--baud",         type=int, default=115200)
    parser.add_argument("--upgrade-baud", type=int, default=0,
                        choices=[0, 230400, 460800, 921600])
    parser.add_argument("--print-every",  type=float, default=1.0)
    parser.add_argument("--snapshot-every", type=float, default=0.05,
                        help="Periodic snapshot interval (s). 0 disables.")
    parser.add_argument("--debug-lines",  type=int, default=0,
                        help="Print first N raw ATMA lines for debugging.")
    parser.add_argument("--filter-code",  type=str, default=None)
    parser.add_argument("--filter-mask",  type=str, default=None)
    parser.add_argument("--preset",       type=str, default=None,
                        choices=sorted(PRESETS.keys()))
    parser.add_argument("--list-signals", action="store_true")
    parser.add_argument("--dump-decoded", action="store_true")
    parser.add_argument("--dump-changed-only", action="store_true")
    parser.add_argument("--show-all-signals",    dest="show_all_signals",    action="store_true")
    parser.add_argument("--selected-all-signals", dest="selected_all_signals", action="store_true")
    parser.add_argument("--no-raw-log",      action="store_true")
    parser.add_argument("--no-decoded-log",  action="store_true")
    parser.add_argument("--no-snapshot-log", action="store_true")
    parser.add_argument("--no-selected-log", action="store_true")
    parser.add_argument("--flush-every",  type=float, default=1.0)
    parser.add_argument("--speed-unit",   choices=["raw", "kph_to_mps", "mps_to_kph"],
                        default="raw")
    parser.add_argument("--max-id-hz",    type=float, default=20.0,
                        help="Max accepted rate per CAN ID. 0 = unlimited.")
    parser.add_argument("--raw-only",     action="store_true",
                        help="Log raw frames only; skip decode/snapshot/selected.")
    parser.add_argument("--stale-ms",     type=float, default=1000.0)
    parser.add_argument("--auto-restart-atma", action="store_true")
    parser.set_defaults(show_all_signals=False, selected_all_signals=False)
    args = parser.parse_args()

    if args.raw_only:
        args.no_decoded_log  = True
        args.no_snapshot_log = True
        args.no_selected_log = True
        args.dump_decoded    = False
        args.dump_changed_only = False
        args.show_all_signals  = False
        args.max_id_hz = 0.0
        print("[INFO] Raw-only mode.")

    if args.preset:
        args.filter_code, args.filter_mask = PRESETS[args.preset]
        print(f"[INFO] Preset '{args.preset}': code={args.filter_code}, mask={args.filter_mask}")

    dbc_path = Path(args.dbc)
    if not dbc_path.exists():
        print(f"[ERROR] DBC not found: {dbc_path}")
        sys.exit(1)

    db = cantools.database.load_file(str(dbc_path), strict=False)
    id_to_msg = {m.frame_id: m for m in db.messages}

    if args.selected_all_signals:
        global SELECTED_ALIASES
        SELECTED_ALIASES = build_all_signal_aliases(db)

    print(f"[INFO] Loaded DBC: {args.dbc}  ({len(db.messages)} messages)")

    if args.list_signals:
        list_signals_and_exit(db)

    if not args.port:
        print("[ERROR] --port is required unless using --list-signals")
        sys.exit(1)

    key_ids = [
        0x130, 0x13C, 0x156, 0x158, 0x17C, 0x184, 0x191, 0x1A3, 0x1A4,
        0x1AB, 0x1B0, 0x1C2, 0x1D0, 0x1E7, 0x1FA, 0x223, 0x255,
        0x309, 0x30C, 0x324, 0x35E, 0x37C, 0x39F,
    ]
    print("[INFO] Known key IDs in DBC:")
    for can_id in sorted(id_to_msg):
        if can_id in key_ids:
            print(f"  0x{can_id:<4X}: {id_to_msg[can_id].name}, len={id_to_msg[can_id].length}")

    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = logs_dir / stamp
    run_dir.mkdir(parents=True, exist_ok=True)
    raw_path = run_dir / "raw.csv"
    decoded_path = run_dir / "decoded.csv"
    snapshot_path = run_dir / "snapshot.csv"
    selected_path = run_dir / "selected.csv"

    # ------------------------------------------------------------------
    # Open serial  –  timeout=0.01 and rtscts=True  (same as Honda)
    # ------------------------------------------------------------------
    print(f"[INFO] Opening {args.port} @ {args.baud}")
    ser = serial.Serial(
        args.port, args.baud,
        timeout=0.01,
        write_timeout=0.2,
        rtscts=True,
    )

    stats = {
        "serial_bytes":        0,
        "serial_lines":        0,
        "atma_restarts":       0,
        "last_restart_reason": "",
    }

    id_counts        = Counter()
    accepted_counts  = Counter()
    throttled_counts = Counter()
    decode_ok        = Counter()
    decode_fail      = Counter()
    fail_reason      = {}

    last_accept_time_by_id = {}
    last_values            = {}
    signal_time            = {}
    prev_decoded_values    = {}

    processed_lines  = 0
    line_debug_count = 0
    last_line_time   = None   # time.perf_counter() of last received serial line

    f_raw = f_dec = f_snap = f_sel = None
    raw_writer = dec_writer = snap_writer = sel_writer = None

    try:
        # ------------------------------------------------------------------
        # AT initialisation
        # ------------------------------------------------------------------
        send_at(ser, "ATZ",   delay=0.5)
        send_at(ser, "ATE0")
        send_at(ser, "ATL0")
        send_at(ser, "ATS0")
        send_at(ser, "ATH1")
        send_at(ser, "ATSP6")

        if args.upgrade_baud > 0:
            ser, active_baud = try_upgrade_baud(ser, args.upgrade_baud)
            args.baud = active_baud
            for cmd in ("ATE0", "ATL0", "ATS0", "ATH1", "ATSP6"):
                send_at(ser, cmd)

        send_at(ser, "ATCAF0")   # raw CAN frames, no formatting

        if args.filter_code and args.filter_mask:
            send_at(ser, f"ATCF{args.filter_code}")
            send_at(ser, f"ATCM{args.filter_mask}")
            print(f"[INFO] CAN filter applied: code=0x{args.filter_code}, mask=0x{args.filter_mask}")
        else:
            # No filter – this is the goal: monitor everything, rely on
            # Honda-style fast restart instead of hardware filtering.
            print("[INFO] No CAN filter. Monitoring all IDs.")

        # ------------------------------------------------------------------
        # Open log files
        # ------------------------------------------------------------------
        if not args.no_raw_log:
            f_raw = open(raw_path, "w", newline="", encoding="utf-8")
            raw_writer = csv.writer(f_raw)
            raw_writer.writerow(["time_wall", "time_perf", "rel_time",
                                  "can_id", "dlc", "data"])

        if not args.no_decoded_log:
            f_dec = open(decoded_path, "w", newline="", encoding="utf-8")
            dec_writer = csv.writer(f_dec)
            dec_writer.writerow(["time_wall", "time_perf", "rel_time",
                                  "can_id", "message", "signal", "full_signal", "value"])

        if not args.no_snapshot_log:
            f_snap = open(snapshot_path, "w", newline="", encoding="utf-8")
            snap_writer = csv.writer(f_snap)
            snap_writer.writerow(make_snapshot_header())

        if not args.no_selected_log:
            f_sel = open(selected_path, "w", newline="", encoding="utf-8")
            sel_writer = csv.writer(f_sel)
            sel_writer.writerow(make_selected_header())

        # ------------------------------------------------------------------
        # Start ATMA
        # ------------------------------------------------------------------
        print("[INFO] Starting ATMA monitor.")
        ser.reset_input_buffer()
        ser.write(b"ATMA\r")
        ser.flush()

        t0_perf  = time.perf_counter()
        last_print    = 0.0
        last_snapshot = 0.0
        last_flush    = 0.0
        last_restart_attempt = 0.0
        buffer = ""

        # ==================================================================
        # Main loop  –  single-threaded, Honda-style
        # ==================================================================
        while True:
            now_perf = time.perf_counter()
            elapsed  = now_perf - t0_perf

            # --- duration check ------------------------------------------
            if args.duration > 0 and elapsed >= args.duration:
                print(f"[INFO] Duration reached: {args.duration}s")
                break

            # --- auto-restart on stale stream ----------------------------
            if args.auto_restart_atma and elapsed - last_restart_attempt > 1.0:
                stale = (
                    (last_line_time is None and elapsed > args.stale_ms / 1000.0)
                    or (last_line_time is not None
                        and (now_perf - last_line_time) * 1000.0 > args.stale_ms)
                )
                if stale:
                    restart_monitor(ser, stats, f"stream stale > {args.stale_ms} ms")
                    last_restart_attempt = elapsed
                    buffer = ""

            # --- read serial (Honda style: in_waiting or 1 byte) ---------
            raw_chunk = ser.read(ser.in_waiting or 1)
            if raw_chunk:
                stats["serial_bytes"] += len(raw_chunk)
                buffer += raw_chunk.decode("ascii", errors="ignore").replace("\r", "\n")

            # --- process every complete line in the buffer ---------------
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                stats["serial_lines"] += 1
                last_line_time = time.perf_counter()

                if args.debug_lines > 0 and line_debug_count < args.debug_lines:
                    print(f"[LINE] {line}")
                    line_debug_count += 1

                parsed = parse_atma_line(line)
                if parsed is None:
                    continue

                # ---- adapter status / prompt ----------------------------
                if parsed[0] == "__PROMPT__":
                    print("[ADAPTER] Prompt '>' detected. ATMA stopped.")
                    if args.auto_restart_atma:
                        restart_monitor(ser, stats, "adapter prompt")
                        buffer = ""
                    continue

                if parsed[0] == "__ADAPTER_STATUS__":
                    status_text = parsed[1].decode("ascii", errors="ignore")
                    print(f"[ADAPTER] {status_text}")
                    # Always restart on BUFFER FULL/STOPPED (Honda behaviour)
                    restart_monitor(ser, stats, status_text)
                    buffer = ""
                    continue

                # ---- valid CAN frame ------------------------------------
                can_id, data  = parsed
                now_wall      = time.time()
                now_perf      = time.perf_counter()
                rel_t         = now_perf - t0_perf
                processed_lines += 1

                id_counts[can_id] += 1

                if not should_accept_id(can_id, now_perf, last_accept_time_by_id, args.max_id_hz):
                    throttled_counts[can_id] += 1
                    continue

                accepted_counts[can_id] += 1

                # raw log
                if raw_writer is not None:
                    raw_writer.writerow([
                        f"{now_wall:.6f}", f"{now_perf:.6f}", f"{rel_t:.6f}",
                        f"0x{can_id:X}", len(data),
                        " ".join(f"{b:02X}" for b in data),
                    ])

                # decode
                if (not args.raw_only) and can_id in id_to_msg:
                    msg = id_to_msg[can_id]
                    pad = msg.length - len(data)
                    data_dec = (data + bytes(pad))[:msg.length] if pad > 0 else data[:msg.length]
                    try:
                        decoded = db.decode_message(can_id, data_dec,
                                                    decode_choices=False, scaling=True)
                        decode_ok[can_id] += 1

                        for sig, val in decoded.items():
                            full_sig = f"{msg.name}.{sig}"
                            last_values[sig]      = val
                            last_values[full_sig] = val
                            signal_time[sig]      = now_perf
                            signal_time[full_sig] = now_perf

                            if args.dump_decoded:
                                old_val = prev_decoded_values.get(full_sig)
                                if (not args.dump_changed_only) or old_val != val:
                                    print(f"[DEC] t={rel_t:.4f}  0x{can_id:X}  "
                                          f"{msg.name}.{sig}={val}")
                                prev_decoded_values[full_sig] = val

                            if dec_writer is not None:
                                dec_writer.writerow([
                                    f"{now_wall:.6f}", f"{now_perf:.6f}", f"{rel_t:.6f}",
                                    f"0x{can_id:X}", msg.name, sig, full_sig, val,
                                ])
                    except Exception as e:
                        decode_fail[can_id] += 1
                        if can_id not in fail_reason:
                            fail_reason[can_id] = str(e)

                # selected log
                if (not args.raw_only) and sel_writer is not None:
                    sel_writer.writerow(make_selected_row(
                        now_wall, now_perf, rel_t, can_id,
                        last_values, signal_time, args.speed_unit,
                    ))

                # periodic snapshot (triggered by each accepted frame)
                if (not args.raw_only) and args.snapshot_every > 0 and snap_writer is not None:
                    if rel_t - last_snapshot >= args.snapshot_every:
                        snap_writer.writerow(make_snapshot_row(
                            now_wall, now_perf, rel_t, last_values, signal_time,
                        ))
                        last_snapshot = rel_t

                # dashboard
                if (not args.raw_only) and rel_t - last_print >= args.print_every:
                    print_dashboard(
                        rel_t, last_values, signal_time,
                        id_counts, accepted_counts, throttled_counts,
                        decode_ok, decode_fail, now_perf,
                        stats, processed_lines, args.stale_ms,
                        args.show_all_signals, last_line_time,
                    )
                    last_print = rel_t

                # periodic flush
                if args.flush_every >= 0 and rel_t - last_flush >= args.flush_every:
                    for f in (f_raw, f_dec, f_snap, f_sel):
                        if f is not None:
                            f.flush()
                    last_flush = rel_t

            # dashboard fires even when bus is silent
            if (not args.raw_only) and elapsed - last_print >= args.print_every:
                print_dashboard(
                    elapsed, last_values, signal_time,
                    id_counts, accepted_counts, throttled_counts,
                    decode_ok, decode_fail, now_perf,
                    stats, processed_lines, args.stale_ms,
                    args.show_all_signals, last_line_time,
                )
                last_print = elapsed

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C – stopping.")

    finally:
        # stop ATMA gracefully
        try:
            ser.write(b" ")
            time.sleep(0.05)
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass

        for f in (f_raw, f_dec, f_snap, f_sel):
            if f is not None:
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass

        # ------------------------------------------------------------------
        # Summary
        # ------------------------------------------------------------------
        print("\n[INFO] Summary:")
        print(f"  raw seen     : {sum(id_counts.values())}")
        print(f"  accepted     : {sum(accepted_counts.values())}")
        print(f"  throttled    : {sum(throttled_counts.values())}")
        print(f"  decoded      : {sum(decode_ok.values())}")
        print(f"  decode fail  : {sum(decode_fail.values())}")
        print(f"  ATMA restarts: {stats.get('atma_restarts', 0)}")
        print(f"  serial bytes : {stats.get('serial_bytes', 0)}")
        if stats.get("last_restart_reason"):
            print(f"  last restart : {stats['last_restart_reason']}")

        print("\n[INFO] Top raw seen IDs:")
        for can_id, cnt in id_counts.most_common(20):
            name = id_to_msg[can_id].name if can_id in id_to_msg else "-"
            print(f"  0x{can_id:<4X}  {cnt:<8}  {name}")

        print("\n[INFO] Top accepted IDs:")
        for can_id, cnt in accepted_counts.most_common(20):
            name = id_to_msg[can_id].name if can_id in id_to_msg else "-"
            print(f"  0x{can_id:<4X}  {cnt:<8}  {name}")

        if decode_fail:
            print("\n[WARN] Decode failures:")
            for can_id, cnt in decode_fail.most_common(20):
                name = id_to_msg[can_id].name if can_id in id_to_msg else "-"
                print(f"  0x{can_id:<4X}  {cnt:<8}  {name}  reason={fail_reason.get(can_id)}")

        print("\n[INFO] Saved:")
        if not args.no_raw_log:      print(f"  raw      : {raw_path}")
        if not args.no_decoded_log:  print(f"  decoded  : {decoded_path}")
        if not args.no_snapshot_log: print(f"  snapshot : {snapshot_path}")
        if not args.no_selected_log: print(f"  selected : {selected_path}")


if __name__ == "__main__":
    main()