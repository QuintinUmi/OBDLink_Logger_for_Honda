import argparse
import csv
import re
import sys
import time
import threading
import queue
from collections import Counter
from datetime import datetime
from pathlib import Path

import cantools
import serial


# ============================================================
# ELM / OBDLink helpers
# ============================================================

def send_at(ser, cmd, delay=0.15, verbose=True, timeout=3.0):
    """
    Send one AT command to ELM327 / OBDLink and read until prompt '>'.
    """
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

    text = buf.decode("ascii", errors="ignore")
    text = text.replace("\r", "\n")
    lines = [x.strip() for x in text.split("\n") if x.strip()]

    if verbose:
        for line in lines:
            if line != cmd:
                print(line)

    time.sleep(delay)
    return text


def stop_monitor(ser):
    """
    Stop ATMA by sending CR, then drain buffer.
    """
    try:
        ser.write(b"\r")
        ser.flush()
        time.sleep(0.1)

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 0.3:
            n = ser.in_waiting
            if n <= 0:
                break

            ser.read(n)
            time.sleep(0.005)

    except Exception:
        pass


def restart_monitor(ser, stats=None, reason=""):
    """
    Restart ATMA without resetting the adapter.
    """
    try:
        if stats is not None:
            stats["atma_restarts"] = stats.get("atma_restarts", 0) + 1
            stats["last_restart_reason"] = reason

        print(f"[WARN] Restart ATMA. reason={reason}")

        ser.write(b"\r")
        ser.flush()
        time.sleep(0.08)

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 0.3:
            n = ser.in_waiting
            if n <= 0:
                break
            ser.read(n)
            time.sleep(0.005)

        ser.write(b"ATMA\r")
        ser.flush()
        return True

    except Exception as e:
        if stats is not None:
            stats["restart_errors"] = stats.get("restart_errors", 0) + 1
            stats["last_restart_error"] = str(e)

        print(f"[ERROR] Failed to restart ATMA: {e}")
        return False


OBDLINK_BAUD_CODES = {
    230400: "34",
    460800: "1A",
    921600: "0D",
}


def try_upgrade_baud(ser, target_baud):
    """
    Try to upgrade OBDLink baud rate via AT BRD (USB only).
    Returns (serial_obj, active_baud).
    """
    if target_baud not in OBDLINK_BAUD_CODES:
        print(f"[WARN] Baud {target_baud} not supported. Options: {sorted(OBDLINK_BAUD_CODES)}")
        return ser, ser.baudrate

    brd_code = OBDLINK_BAUD_CODES[target_baud]
    port = ser.port
    original_baud = ser.baudrate

    print(f"[INFO] Attempting baud upgrade {original_baud} -> {target_baud} (AT BRD {brd_code})")

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

        new_ser = serial.Serial(port, target_baud, timeout=0, write_timeout=0.5)

        def wait_for_prompt(timeout_s):
            t_start = time.perf_counter()
            buf_local = b""
            while time.perf_counter() - t_start < timeout_s:
                n = new_ser.in_waiting
                if n > 0:
                    buf_local += new_ser.read(n)
                    if b">" in buf_local or b"OK" in buf_local:
                        return True, buf_local
                time.sleep(0.003)
            return False, buf_local

        # Try to resync prompt after baud change.
        for _ in range(3):
            new_ser.write(b"\r")
            new_ser.flush()
            ok, buf = wait_for_prompt(0.6)
            if ok:
                print(f"[INFO] Baud upgrade OK -> {target_baud}.")
                return new_ser, target_baud

        # Try an explicit AT to elicit OK/prompt.
        new_ser.write(b"AT\r")
        new_ser.flush()
        ok, buf = wait_for_prompt(1.2)
        if ok:
            print(f"[INFO] Baud upgrade OK -> {target_baud}.")
            return new_ser, target_baud

        print(f"[WARN] Baud upgrade confirmation failed ({buf!r}). Rolling back.")
        new_ser.close()
        time.sleep(0.1)
        recovered = serial.Serial(port, original_baud, timeout=0, write_timeout=0.2)
        send_at(recovered, "ATZ", delay=0.5, verbose=False)
        return recovered, original_baud

    except Exception as e:
        print(f"[WARN] Baud upgrade exception: {e}")
        try:
            fallback = serial.Serial(port, original_baud, timeout=0, write_timeout=0.2)
            return fallback, original_baud
        except Exception:
            return ser, original_baud


# ============================================================
# CAN line parser
# ============================================================

HEX_PAIR_RE = re.compile(r"^[0-9A-Fa-f]{2}$")
HEX_ID_RE = re.compile(r"^[0-9A-Fa-f]{1,8}$")


def parse_atma_line(line):
    """
    Parse common OBDLink/ELM ATMA text lines.

    Examples:
      "1AB 00 00 11"
      "130 00 00 00 00 00 00 04 2E"
      "18DAF110 03 41 0C 1A F8"

    Returns:
      (can_id:int, data:bytes)
      ("__PROMPT__", b"")
      ("__ADAPTER_STATUS__", b"...")
      or None
    """
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

    adapter_status_words = [
        "NO DATA",
        "STOPPED",
        "BUFFER FULL",
        "CAN ERROR",
        "BUS ERROR",
        "UNABLE TO CONNECT",
    ]

    if any(x in upper for x in adapter_status_words):
        return ("__ADAPTER_STATUS__", upper.encode("ascii", errors="ignore"))

    if upper in ["OK", "?"]:
        return None

    if "SEARCHING" in upper:
        return None

    raw_tokens = re.split(r"[\s,]+", s)
    tokens = []

    for tok in raw_tokens:
        tok = tok.strip()
        if not tok:
            continue

        tok = tok.replace(":", "")

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

    # Handle optional DLC token format:
    #   "130 8 00 00 00 00 00 00 04 2E"
    if len(data_tokens) >= 2:
        try:
            maybe_dlc = int(data_tokens[0], 16)
            remaining = data_tokens[1:]

            if (
                0 <= maybe_dlc <= 8
                and len(remaining) == maybe_dlc
                and all(HEX_PAIR_RE.match(x) for x in remaining)
            ):
                data_tokens = remaining

        except Exception:
            pass

    data_bytes = []

    for tok in data_tokens:
        if len(tok) != 2:
            continue

        if not HEX_PAIR_RE.match(tok):
            continue

        data_bytes.append(int(tok, 16))

    if len(data_bytes) == 0:
        return None

    return can_id, bytes(data_bytes[:8])


# ============================================================
# Low-latency async serial reader
# ============================================================

class SerialReaderThread(threading.Thread):
    """
    Dedicated low-latency serial reader.

    The reader only reads serial bytes and pushes complete text lines
    to a queue. Parsing/decoding/logging are done in the main thread.
    """

    def __init__(self, ser, line_queue, stop_event, stats, sleep_empty=0.0005):
        super().__init__(daemon=True)

        self.ser = ser
        self.line_queue = line_queue
        self.stop_event = stop_event
        self.stats = stats
        self.sleep_empty = sleep_empty
        self.buf = ""

    def push_line_drop_oldest(self, item):
        try:
            self.line_queue.put_nowait(item)

        except queue.Full:
            self.stats["queue_dropped_oldest"] += 1

            try:
                self.line_queue.get_nowait()
            except queue.Empty:
                pass

            try:
                self.line_queue.put_nowait(item)
            except queue.Full:
                self.stats["queue_put_failed"] += 1

    def run(self):
        while not self.stop_event.is_set():
            try:
                n = self.ser.in_waiting

                if n > 0:
                    chunk = self.ser.read(n)
                else:
                    chunk = self.ser.read(1)

                if not chunk:
                    time.sleep(self.sleep_empty)
                    continue

                self.stats["serial_bytes"] += len(chunk)

                text = chunk.decode("ascii", errors="ignore")
                text = text.replace("\r", "\n")
                self.buf += text

                while "\n" in self.buf:
                    line, self.buf = self.buf.split("\n", 1)
                    line = line.strip()

                    if not line:
                        continue

                    t_arrival = time.perf_counter()
                    self.stats["serial_lines"] += 1
                    self.stats["last_serial_line_time"] = t_arrival

                    self.push_line_drop_oldest((t_arrival, line))

            except Exception as e:
                self.stats["reader_errors"] += 1
                self.stats["last_reader_error"] = str(e)
                time.sleep(0.005)

        if self.buf.strip():
            t_arrival = time.perf_counter()
            self.stats["serial_lines"] += 1
            self.stats["last_serial_line_time"] = t_arrival
            self.push_line_drop_oldest((t_arrival, self.buf.strip()))
            self.buf = ""


# ============================================================
# Signal presets / aliases
# ============================================================

PRESETS = {
    # Stable narrow filters.
    "steer_rpm": ("150", "7F0"),       # 0x150 - 0x15F: 0x156, 0x158
    "steer": ("156", "7FF"),           # 0x156
    "rpm": ("158", "7FF"),             # 0x158

    "wheel": ("1D0", "7FF"),           # 0x1D0 WHEEL_SPEEDS
    "rough_wheel": ("255", "7FF"),     # 0x255 ROUGH_WHEEL_SPEED

    "gas": ("130", "7FF"),             # 0x130 GAS_PEDAL_2
    "gas2": ("13C", "7FF"),            # 0x13C GAS_PEDAL

    "brake": ("17C", "7FF"),           # 0x17C POWERTRAIN_DATA
    "brake_pressure": ("1E7", "7FF"),  # 0x1E7 BRAKE_PRESSURE
    "vsa": ("1A4", "7FF"),             # 0x1A4 VSA_STATUS

    "car_speed": ("309", "7FF"),       # 0x309 CAR_SPEED

    # Gear.
    "gear_191": ("191", "7FF"),

    # Wider scan filters. May cause BUFFER FULL/STOPPED on ELM-style devices.
    "speed_related_100": ("100", "700"),
    "speed_related_200": ("200", "700"),
    "speed_related_300": ("300", "700"),

    "range_000": ("000", "700"),
    "range_100": ("100", "700"),
    "range_200": ("200", "700"),
    "range_300": ("300", "700"),
    "range_400": ("400", "700"),
}


DEFAULT_ROTATION_GROUPS = [
    {"code": "156", "mask": "7FF", "duration": 0.35, "label": "steer"},
    {"code": "1D0", "mask": "7FF", "duration": 0.30, "label": "wheel_speeds"},
    {"code": "17C", "mask": "7FF", "duration": 0.25, "label": "brake"},
    {"code": "191", "mask": "7FF", "duration": 0.20, "label": "gear"},
    {"code": "130", "mask": "7F0", "duration": 0.25, "label": "gas"},
    {"code": "309", "mask": "7FF", "duration": 0.20, "label": "car_speed"},
]


class FilterRotator:
    _RUNNING = "running"
    _STOPPING = "stopping"
    _APPLYING_CF = "apply_cf"
    _APPLYING_CM = "apply_cm"
    _RESTARTING = "restarting"

    _DELAY_STOP = 0.065
    _DELAY_CF = 0.050
    _DELAY_CM = 0.060

    def __init__(self, groups, ser, stats=None):
        self.groups = groups
        self.ser = ser
        self.stats = stats
        self.idx = 0
        self._state = self._RUNNING
        self._state_ts = None
        self._group_start = None

    @property
    def current_label(self):
        return self.groups[self.idx]["label"]

    def _drain(self):
        try:
            n = self.ser.in_waiting
            if n > 0:
                self.ser.read(n)
        except Exception:
            pass

    def tick(self, now_perf):
        if self._group_start is None:
            self._group_start = now_perf

        g = self.groups[self.idx]

        if self._state == self._RUNNING:
            if (now_perf - self._group_start) >= g["duration"]:
                self.ser.write(b"\r")
                self.ser.flush()
                self._state = self._STOPPING
                self._state_ts = now_perf
            return False

        if self._state_ts is None:
            self._state_ts = now_perf
            return False

        elapsed = now_perf - self._state_ts

        if self._state == self._STOPPING and elapsed >= self._DELAY_STOP:
            self._drain()
            self.idx = (self.idx + 1) % len(self.groups)
            ng = self.groups[self.idx]
            self.ser.write(f"ATCF{ng['code']}\r".encode("ascii", errors="ignore"))
            self.ser.flush()
            self._state = self._APPLYING_CF
            self._state_ts = now_perf

        elif self._state == self._APPLYING_CF and elapsed >= self._DELAY_CF:
            self._drain()
            ng = self.groups[self.idx]
            self.ser.write(f"ATCM{ng['mask']}\r".encode("ascii", errors="ignore"))
            self.ser.flush()
            self._state = self._APPLYING_CM
            self._state_ts = now_perf

        elif self._state == self._APPLYING_CM and elapsed >= self._DELAY_CM:
            self._drain()
            self.ser.write(b"ATMA\r")
            self.ser.flush()
            self._state = self._RESTARTING
            self._state_ts = now_perf

        elif self._state == self._RESTARTING:
            self._state = self._RUNNING
            self._group_start = now_perf
            ng = self.groups[self.idx]
            if self.stats is not None:
                self.stats["atma_restarts"] = self.stats.get("atma_restarts", 0) + 1
                self.stats["last_restart_reason"] = f"rotate:{ng['label']}"
            print(f"[ROTATE] Filter: {ng['code']}/{ng['mask']} ({ng['label']})")
            return True

        return False


WATCH_SIGNALS = [
    ("VEHICLE_SPEED", [
        "CAR_SPEED.ROUGH_CAR_SPEED_3",
        "ROUGH_CAR_SPEED_3",
        "CAR_SPEED.CAR_SPEED",
        "CAR_SPEED",
    ]),

    ("BRAKE_ACTIVE", [
        "POWERTRAIN_DATA.BRAKE_SWITCH",
        "BRAKE_SWITCH",
        "POWERTRAIN_DATA.BRAKE_PRESSED",
        "BRAKE_PRESSED",
        "VSA_STATUS.USER_BRAKE",
        "USER_BRAKE",
    ]),


    ("STEER_ANGLE", [
        "STEERING_SENSORS.STEER_ANGLE",
        "STEER_ANGLE",
    ]),


    ("WHEEL_SPEED_FL", [
        "WHEEL_SPEEDS.WHEEL_SPEED_FL",
        "ROUGH_WHEEL_SPEED.WHEEL_SPEED_FL",
        "WHEEL_SPEED_FL",
    ]),
    ("WHEEL_SPEED_FR", [
        "WHEEL_SPEEDS.WHEEL_SPEED_FR",
        "ROUGH_WHEEL_SPEED.WHEEL_SPEED_FR",
        "WHEEL_SPEED_FR",
    ]),
    ("WHEEL_SPEED_RL", [
        "WHEEL_SPEEDS.WHEEL_SPEED_RL",
        "ROUGH_WHEEL_SPEED.WHEEL_SPEED_RL",
        "WHEEL_SPEED_RL",
    ]),
    ("WHEEL_SPEED_RR", [
        "WHEEL_SPEEDS.WHEEL_SPEED_RR",
        "ROUGH_WHEEL_SPEED.WHEEL_SPEED_RR",
        "WHEEL_SPEED_RR",
    ]),

    
]


# ============================================================
# Display / snapshot helpers
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
    elapsed,
    last_values,
    signal_time,
    id_counts,
    accepted_counts,
    throttled_counts,
    decode_ok,
    decode_fail,
    now_perf,
    line_queue,
    reader_stats,
    processed_lines,
    stale_ms,
    show_all_signals,
):
    print("-" * 116)
    print(
        f"t={elapsed:.2f}s | "
        f"raw_seen={sum(id_counts.values())} | "
        f"accepted={sum(accepted_counts.values())} | "
        f"throttled={sum(throttled_counts.values())} | "
        f"decoded_ok={sum(decode_ok.values())} | "
        f"decoded_fail={sum(decode_fail.values())} | "
        f"queue={line_queue.qsize()} | "
        f"processed_lines={processed_lines} | "
        f"serial_lines={reader_stats.get('serial_lines', 0)} | "
        f"dropped={reader_stats.get('queue_dropped_oldest', 0)} | "
        f"restarts={reader_stats.get('atma_restarts', 0)}"
    )

    last_line_t = reader_stats.get("last_serial_line_time")

    if last_line_t is None:
        print("Serial stream        : NO LINE YET")
    else:
        stream_age_ms = (now_perf - last_line_t) * 1000.0
        stream_state = "STALE" if stream_age_ms > stale_ms else "LIVE"
        print(f"Serial stream        : {stream_state}, age={stream_age_ms:.1f} ms")

    for label, aliases in WATCH_SIGNALS:
        v, age_ms = get_value_and_age_ms(last_values, signal_time, aliases, now_perf)

        if age_ms is None:
            print(f"{label:<20}: {fmt_value(v):<14} age=NA          STALE")
        else:
            state = "STALE" if age_ms > stale_ms else "LIVE"
            print(f"{label:<20}: {fmt_value(v):<14} age={age_ms:8.1f} ms  {state}")

    if id_counts:
        top = " ".join([f"0x{k:X}:{v}" for k, v in id_counts.most_common(10)])
        print(f"Top raw seen IDs     : {top}")

    if accepted_counts:
        top = " ".join([f"0x{k:X}:{v}" for k, v in accepted_counts.most_common(10)])
        print(f"Top accepted IDs     : {top}")

    if throttled_counts:
        top = " ".join([f"0x{k:X}:{v}" for k, v in throttled_counts.most_common(10)])
        print(f"Top throttled IDs    : {top}")

    if decode_fail:
        top_fail = " ".join([f"0x{k:X}:{v}" for k, v in decode_fail.most_common(5)])
        print(f"Decode fail IDs      : {top_fail}")

    if reader_stats.get("last_restart_reason"):
        print(f"Last restart reason  : {reader_stats.get('last_restart_reason')}")

    if reader_stats.get("reader_errors", 0) > 0:
        print(f"Reader errors        : {reader_stats.get('reader_errors')} last={reader_stats.get('last_reader_error')}")

    if show_all_signals:
        full_keys = sorted(k for k in last_values.keys() if "." in k)
        if full_keys:
            print("All decoded signals  :")
            for key in full_keys:
                print(f"  {key}={fmt_value(last_values.get(key))}")

    print()


def make_snapshot_header():
    header = ["time_wall", "time_perf", "rel_time"]

    for label, _aliases in WATCH_SIGNALS:
        header.append(label)
        header.append(label + "_age_ms")

    return header


def make_snapshot_row(now_wall, now_perf, rel_time, last_values, signal_time):
    row = [
        f"{now_wall:.6f}",
        f"{now_perf:.6f}",
        f"{rel_time:.6f}",
    ]

    for _label, aliases in WATCH_SIGNALS:
        v, age_ms = get_value_and_age_ms(last_values, signal_time, aliases, now_perf)
        row.append(format_csv_value(v))
        row.append("" if age_ms is None else f"{age_ms:.3f}")

    return row


def list_signals_and_exit(db):
    keywords = [
        "SPEED",
        "WHEEL",
        "BRAKE",
        "BLINKER",
        "TURN",
        "SIGNAL",
        "LAMP",
        "LIGHT",
        "LEFT",
        "RIGHT",
        "STALK",
        "STEER",
        "GAS",
        "PEDAL",
        "RPM",
        "XMISSION",
        "GEAR",
    ]

    print("\n[INFO] Signals matched by keywords:")

    for msg in db.messages:
        for sig in msg.signals:
            name_upper = sig.name.upper()
            msg_upper = msg.name.upper()

            if any(k in name_upper or k in msg_upper for k in keywords):
                print(
                    f"  ID=0x{msg.frame_id:<4X} "
                    f"MSG={msg.name:<35} "
                    f"SIG={sig.name:<35} "
                    f"LEN={msg.length}"
                )

    sys.exit(0)


# ============================================================
# Selected signal helpers
# ============================================================

SELECTED_ALIASES = {
    "vehicle_speed": [
        "CAR_SPEED.ROUGH_CAR_SPEED_3",
        "ROUGH_CAR_SPEED_3",
        "CAR_SPEED.CAR_SPEED",
        "CAR_SPEED",
    ],
    "gear": [
        "GEARBOX_CVT.GEAR_SHIFTER",
        "GEARBOX_AUTO.GEAR_SHIFTER",
        "GEARBOX.GEAR_SHIFTER",
        "GEAR_SHIFTER",
    ],
    "gas_pedal": [
        "GAS_PEDAL_2.CAR_GAS",
        "GAS_PEDAL.CAR_GAS",
        "CAR_GAS",
        "POWERTRAIN_DATA.PEDAL_GAS",
        "PEDAL_GAS",
    ],
    "brake_active": [
        "POWERTRAIN_DATA.BRAKE_SWITCH",
        "BRAKE_SWITCH",
        "POWERTRAIN_DATA.BRAKE_PRESSED",
        "BRAKE_PRESSED",
        "VSA_STATUS.USER_BRAKE",
        "USER_BRAKE",
    ],
    "steer_angle": [
        "STEERING_SENSORS.STEER_ANGLE",
        "STEER_ANGLE",
    ],
    "wheel_speed_fl": [
        "WHEEL_SPEEDS.WHEEL_SPEED_FL",
        "ROUGH_WHEEL_SPEED.WHEEL_SPEED_FL",
        "WHEEL_SPEED_FL",
    ],
    "wheel_speed_fr": [
        "WHEEL_SPEEDS.WHEEL_SPEED_FR",
        "ROUGH_WHEEL_SPEED.WHEEL_SPEED_FR",
        "WHEEL_SPEED_FR",
    ],
    "wheel_speed_rl": [
        "WHEEL_SPEEDS.WHEEL_SPEED_RL",
        "ROUGH_WHEEL_SPEED.WHEEL_SPEED_RL",
        "WHEEL_SPEED_RL",
    ],
    "wheel_speed_rr": [
        "WHEEL_SPEEDS.WHEEL_SPEED_RR",
        "ROUGH_WHEEL_SPEED.WHEEL_SPEED_RR",
        "WHEEL_SPEED_RR",
    ],
}


def convert_selected_value(name, value, speed_unit):
    if value is None:
        return None

    if name in {
        "vehicle_speed",
        "wheel_speed_fl",
        "wheel_speed_fr",
        "wheel_speed_rl",
        "wheel_speed_rr",
    }:
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
    header = [
        "time_wall",
        "time_perf",
        "arrival_perf",
        "rel_time",
        "trigger_can_id",
    ]

    for name in SELECTED_ALIASES.keys():
        header.append(name)
        header.append(name + "_age_ms")

    return header


def make_selected_row(
    now_wall,
    now_perf,
    arrival_perf,
    rel_t,
    trigger_can_id,
    last_values,
    signal_time,
    speed_unit,
):
    row = [
        f"{now_wall:.6f}",
        f"{now_perf:.6f}",
        f"{arrival_perf:.6f}",
        f"{rel_t:.6f}",
        f"0x{trigger_can_id:X}",
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
    """
    Limit accepted processing rate per CAN ID.

    max_id_hz <= 0 means no rate limit.
    """
    if max_id_hz is None or max_id_hz <= 0:
        last_accept_time_by_id[can_id] = now_perf
        return True

    min_dt = 1.0 / max_id_hz
    last_t = last_accept_time_by_id.get(can_id)

    if last_t is None or (now_perf - last_t) >= min_dt:
        last_accept_time_by_id[can_id] = now_perf
        return True

    return False


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--port", required=False, help="Serial port, e.g. COM10")
    parser.add_argument("--dbc", required=True, help="DBC file path")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to monitor. Use 0 for infinite until Ctrl+C.",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    parser.add_argument(
        "--upgrade-baud",
        type=int,
        default=0,
        choices=[0, 230400, 460800, 921600],
        help="Upgrade OBDLink baud via AT BRD (USB only). 0 disables.",
    )

    parser.add_argument("--all", action="store_true", help="Use ATMA monitor all")
    parser.add_argument("--print-every", type=float, default=1.0, help="Dashboard print interval")
    parser.add_argument(
        "--snapshot-every",
        type=float,
        default=0.05,
        help="Snapshot interval in seconds. 0.05 = 20 Hz. Set 0 to disable periodic snapshot.",
    )

    parser.add_argument("--debug-lines", type=int, default=0, help="Print first N raw ATMA lines")
    parser.add_argument("--filter-code", type=str, default=None, help="ELM CAN filter code, e.g. 100, 150, 156")
    parser.add_argument("--filter-mask", type=str, default=None, help="ELM CAN filter mask, e.g. 700, 7F0, 7FF")
    parser.add_argument("--preset", type=str, default=None, choices=sorted(PRESETS.keys()), help="Predefined CAN filter preset")

    parser.add_argument("--list-signals", action="store_true", help="List useful signals in DBC and exit")
    parser.add_argument("--dump-decoded", action="store_true", help="Print decoded signals live")
    parser.add_argument("--dump-changed-only", action="store_true", help="Only print changed decoded signals")
    parser.add_argument(
        "--rotate",
        action="store_true",
        help="Rotate through narrow CAN filters to reduce buffer pressure.",
    )
    parser.add_argument(
        "--show-all-signals",
        dest="show_all_signals",
        action="store_true",
        help="Print all decoded signals on the dashboard.",
    )
    parser.add_argument(
        "--selected-all-signals",
        dest="selected_all_signals",
        action="store_true",
        help="Add all decoded signals to selected log.",
    )

    parser.add_argument(
        "--queue-size",
        type=int,
        default=2000,
        help="Max Python-side line queue size. If full, drop oldest lines.",
    )
    parser.add_argument(
        "--no-raw-log",
        action="store_true",
        help="Disable accepted raw CSV logging for lower latency.",
    )
    parser.add_argument(
        "--no-decoded-log",
        action="store_true",
        help="Disable decoded CSV logging for lower latency.",
    )
    parser.add_argument(
        "--no-snapshot-log",
        action="store_true",
        help="Disable snapshot CSV logging for lower latency.",
    )
    parser.add_argument(
        "--no-selected-log",
        action="store_true",
        help="Disable selected CSV logging for lower latency.",
    )
    parser.add_argument(
        "--flush-every",
        type=float,
        default=1.0,
        help="CSV flush interval in seconds. Use 0 for every write, larger for lower overhead.",
    )
    parser.add_argument(
        "--speed-unit",
        choices=["raw", "kph_to_mps", "mps_to_kph"],
        default="raw",
        help="Optional conversion for selected VEHICLE_SPEED and WHEEL_SPEED values.",
    )

    parser.add_argument(
        "--max-id-hz",
        type=float,
        default=20.0,
        help="Max accepted processing/logging rate per CAN ID. Default 20 Hz. Use 0 to disable.",
    )

    parser.add_argument(
        "--stale-ms",
        type=float,
        default=1000.0,
        help="If no new serial line for this time, stream is stale.",
    )
    parser.add_argument(
        "--auto-restart-atma",
        action="store_true",
        help="Automatically restart ATMA if stream becomes stale or adapter STOPPED/BUFFER FULL appears.",
    )
    parser.add_argument(
        "--use-atst00",
        action="store_true",
        help="Force sending ATST00. Not recommended if ATMA stops quickly.",
    )

    parser.set_defaults(show_all_signals=False, selected_all_signals=False)
    args = parser.parse_args()

    if args.preset:
        preset_code, preset_mask = PRESETS[args.preset]

        if args.filter_code or args.filter_mask:
            print("[WARN] Both --preset and --filter-code/--filter-mask are provided. --preset will be used.")

        args.filter_code = preset_code
        args.filter_mask = preset_mask

        print(
            f"[INFO] Using preset '{args.preset}': "
            f"filter-code={args.filter_code}, filter-mask={args.filter_mask}"
        )

    dbc_path = Path(args.dbc)

    if not dbc_path.exists():
        print(f"[ERROR] DBC not found: {dbc_path}")
        sys.exit(1)

    db = cantools.database.load_file(str(dbc_path), strict=False)
    id_to_msg = {m.frame_id: m for m in db.messages}

    if args.selected_all_signals:
        global SELECTED_ALIASES
        SELECTED_ALIASES = build_all_signal_aliases(db)

    print(f"[INFO] Loaded DBC: {args.dbc}")
    print(f"[INFO] DBC messages: {len(db.messages)}")

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

    print("[INFO] Known key IDs:")
    for can_id in sorted(id_to_msg.keys()):
        if can_id in key_ids:
            print(f"  0x{can_id:<4X}: {id_to_msg[can_id].name}, len={id_to_msg[can_id].length}")

    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = logs_dir / f"raw_async_{stamp}.csv"
    decoded_path = logs_dir / f"decoded_async_{stamp}.csv"
    snapshot_path = logs_dir / f"snapshot_async_{stamp}.csv"
    selected_path = logs_dir / f"selected_async_{stamp}.csv"

    print(f"[INFO] Opening {args.port} @ {args.baud}")

    ser = serial.Serial(
        args.port,
        args.baud,
        timeout=0,
        write_timeout=0.2,
    )

    stop_event = threading.Event()
    line_queue = queue.Queue(maxsize=max(1, args.queue_size))

    reader_stats = {
        "serial_bytes": 0,
        "serial_lines": 0,
        "last_serial_line_time": None,
        "queue_dropped_oldest": 0,
        "queue_put_failed": 0,
        "reader_errors": 0,
        "last_reader_error": "",
        "atma_restarts": 0,
        "restart_errors": 0,
        "last_restart_reason": "",
        "last_restart_error": "",
    }

    reader = None

    id_counts = Counter()
    accepted_counts = Counter()
    throttled_counts = Counter()
    decode_ok = Counter()
    decode_fail = Counter()
    fail_reason = {}

    last_accept_time_by_id = {}
    last_values = {}
    signal_time = {}
    prev_decoded_values = {}

    line_debug_count = 0
    processed_lines = 0

    f_raw = None
    f_dec = None
    f_snap = None
    f_sel = None

    raw_writer = None
    dec_writer = None
    snap_writer = None
    sel_writer = None

    try:
        # ------------------------------------------------------------
        # Initialization
        # ------------------------------------------------------------
        send_at(ser, "ATZ", delay=0.5)
        send_at(ser, "ATE0")
        send_at(ser, "ATL0")
        send_at(ser, "ATS0")
        send_at(ser, "ATH1")
        send_at(ser, "ATSP6")

        if args.upgrade_baud > 0:
            ser, active_baud = try_upgrade_baud(ser, args.upgrade_baud)
            args.baud = active_baud
            send_at(ser, "ATE0")
            send_at(ser, "ATL0")
            send_at(ser, "ATS0")
            send_at(ser, "ATH1")
            send_at(ser, "ATSP6")

        send_at(ser, "ATCAF0", verbose=True)
        send_at(ser, "ATCFC0", verbose=True)

        send_at(ser, "ATAT0", verbose=True)

        if args.use_atst00:
            send_at(ser, "ATST00", verbose=True)
        else:
            print("[INFO] Skip ATST00 to avoid short ATMA burst/timeout.")

        rotator = None
        if args.rotate:
            if args.filter_code or args.filter_mask or args.preset:
                print("[WARN] --rotate ignores --preset/--filter-code/--filter-mask.")

            rotator = FilterRotator(DEFAULT_ROTATION_GROUPS, ser, reader_stats)
            first_group = DEFAULT_ROTATION_GROUPS[0]
            send_at(ser, f"ATCF{first_group['code']}", verbose=True)
            send_at(ser, f"ATCM{first_group['mask']}", verbose=True)
            print(
                f"[ROTATE] Starting with: {first_group['code']}/{first_group['mask']} "
                f"({first_group['label']})"
            )
        elif args.filter_code and args.filter_mask:
            send_at(ser, f"ATCF{args.filter_code}", verbose=True)
            send_at(ser, f"ATCM{args.filter_mask}", verbose=True)
            print(f"[INFO] Applied CAN filter: code=0x{args.filter_code}, mask=0x{args.filter_mask}")
        else:
            print("[INFO] No CAN filter applied. WARNING: ATMA all may overflow ELM/OBDLink buffer.")

        print("[INFO] Start monitor: ATMA")
        ser.reset_input_buffer()
        ser.write(b"ATMA\r")
        ser.flush()

        reader = SerialReaderThread(
            ser=ser,
            line_queue=line_queue,
            stop_event=stop_event,
            stats=reader_stats,
            sleep_empty=0.0005,
        )
        reader.start()

        # ------------------------------------------------------------
        # Open logs
        # ------------------------------------------------------------
        if not args.no_raw_log:
            f_raw = open(raw_path, "w", newline="", encoding="utf-8")
            raw_writer = csv.writer(f_raw)
            raw_writer.writerow([
                "time_wall",
                "time_perf",
                "arrival_perf",
                "rel_time",
                "can_id",
                "dlc",
                "data",
            ])

        if not args.no_decoded_log:
            f_dec = open(decoded_path, "w", newline="", encoding="utf-8")
            dec_writer = csv.writer(f_dec)
            dec_writer.writerow([
                "time_wall",
                "time_perf",
                "arrival_perf",
                "rel_time",
                "can_id",
                "message",
                "signal",
                "full_signal",
                "value",
            ])

        if not args.no_snapshot_log:
            f_snap = open(snapshot_path, "w", newline="", encoding="utf-8")
            snap_writer = csv.writer(f_snap)
            snap_writer.writerow(make_snapshot_header())

        if not args.no_selected_log:
            f_sel = open(selected_path, "w", newline="", encoding="utf-8")
            sel_writer = csv.writer(f_sel)
            sel_writer.writerow(make_selected_header())

        # ------------------------------------------------------------
        # Main loop
        # ------------------------------------------------------------
        t0_perf = time.perf_counter()
        last_print = 0.0
        last_snapshot = 0.0
        last_flush = 0.0
        last_restart_attempt = 0.0

        try:
            while True:
                now_perf = time.perf_counter()
                elapsed = now_perf - t0_perf

                if args.duration > 0 and elapsed >= args.duration:
                    break

                # Auto restart ATMA if stale.
                if args.auto_restart_atma:
                    last_line_t = reader_stats.get("last_serial_line_time")
                    stream_stale = False

                    if last_line_t is None:
                        if elapsed > args.stale_ms / 1000.0:
                            stream_stale = True
                    else:
                        if (now_perf - last_line_t) * 1000.0 > args.stale_ms:
                            stream_stale = True

                    if stream_stale and elapsed - last_restart_attempt > 1.0:
                        restart_monitor(
                            ser,
                            reader_stats,
                            reason=f"stream stale > {args.stale_ms} ms",
                        )
                        last_restart_attempt = elapsed

                try:
                    arrival_perf, line = line_queue.get(timeout=0.002)

                except queue.Empty:
                    now_perf = time.perf_counter()
                    elapsed = now_perf - t0_perf

                    # Periodic snapshot even if no data.
                    if args.snapshot_every > 0 and (not args.no_snapshot_log) and snap_writer is not None:
                        if elapsed - last_snapshot >= args.snapshot_every:
                            snap_writer.writerow(
                                make_snapshot_row(
                                    time.time(),
                                    now_perf,
                                    elapsed,
                                    last_values,
                                    signal_time,
                                )
                            )
                            last_snapshot = elapsed

                    # Periodic dashboard.
                    if elapsed - last_print >= args.print_every:
                        print_dashboard(
                            elapsed,
                            last_values,
                            signal_time,
                            id_counts,
                            accepted_counts,
                            throttled_counts,
                            decode_ok,
                            decode_fail,
                            now_perf,
                            line_queue,
                            reader_stats,
                            processed_lines,
                            args.stale_ms,
                            args.show_all_signals,
                        )
                        last_print = elapsed

                    if rotator is not None:
                        rotator.tick(now_perf)

                    continue

                processed_lines += 1

                if args.debug_lines > 0 and line_debug_count < args.debug_lines:
                    print(f"[LINE] {line}")
                    line_debug_count += 1

                parsed = parse_atma_line(line)

                if parsed is None:
                    continue

                # Adapter prompt/status handling.
                if parsed[0] == "__PROMPT__":
                    print("[ADAPTER] Prompt '>' detected. ATMA likely stopped.")
                    if args.auto_restart_atma:
                        restart_monitor(ser, reader_stats, reason="adapter prompt")
                    continue

                if parsed[0] == "__ADAPTER_STATUS__":
                    status_text = parsed[1].decode("ascii", errors="ignore")
                    print(f"[ADAPTER] {status_text}")

                    if args.auto_restart_atma:
                        restart_monitor(ser, reader_stats, reason=status_text)

                    continue

                can_id, data = parsed

                now_wall = time.time()
                now_perf = time.perf_counter()
                rel_t = now_perf - t0_perf

                # Count every seen raw CAN line before throttling.
                id_counts[can_id] += 1

                # Per-ID 20Hz rate limit.
                if not should_accept_id(
                    can_id,
                    now_perf,
                    last_accept_time_by_id,
                    args.max_id_hz,
                ):
                    throttled_counts[can_id] += 1
                    continue

                accepted_counts[can_id] += 1

                if raw_writer is not None:
                    raw_writer.writerow([
                        f"{now_wall:.6f}",
                        f"{now_perf:.6f}",
                        f"{arrival_perf:.6f}",
                        f"{rel_t:.6f}",
                        f"0x{can_id:X}",
                        len(data),
                        " ".join(f"{b:02X}" for b in data),
                    ])

                # Decode if in DBC.
                if can_id in id_to_msg:
                    msg = id_to_msg[can_id]

                    if len(data) < msg.length:
                        data_decode = data + bytes(msg.length - len(data))
                    else:
                        data_decode = data[:msg.length]

                    try:
                        decoded = db.decode_message(
                            can_id,
                            data_decode,
                            decode_choices=False,
                            scaling=True,
                        )
                        decode_ok[can_id] += 1

                        for sig, val in decoded.items():
                            full_sig = f"{msg.name}.{sig}"

                            last_values[sig] = val
                            last_values[full_sig] = val
                            signal_time[sig] = now_perf
                            signal_time[full_sig] = now_perf

                            if args.dump_decoded:
                                key = full_sig
                                old_val = prev_decoded_values.get(key, None)
                                changed = old_val != val

                                if (not args.dump_changed_only) or changed:
                                    print(
                                        f"[DEC] t={rel_t:.6f} "
                                        f"id=0x{can_id:X} "
                                        f"msg={msg.name} "
                                        f"sig={sig} "
                                        f"val={val}"
                                    )

                                prev_decoded_values[key] = val

                            if dec_writer is not None:
                                dec_writer.writerow([
                                    f"{now_wall:.6f}",
                                    f"{now_perf:.6f}",
                                    f"{arrival_perf:.6f}",
                                    f"{rel_t:.6f}",
                                    f"0x{can_id:X}",
                                    msg.name,
                                    sig,
                                    full_sig,
                                    val,
                                ])

                    except Exception as e:
                        decode_fail[can_id] += 1

                        if can_id not in fail_reason:
                            fail_reason[can_id] = str(e)

                # Selected signal output, also limited by accepted CAN ID rate.
                if sel_writer is not None:
                    sel_writer.writerow(
                        make_selected_row(
                            now_wall,
                            now_perf,
                            arrival_perf,
                            rel_t,
                            can_id,
                            last_values,
                            signal_time,
                            args.speed_unit,
                        )
                    )

                # Periodic snapshot.
                if args.snapshot_every > 0 and (not args.no_snapshot_log) and snap_writer is not None:
                    if rel_t - last_snapshot >= args.snapshot_every:
                        snap_writer.writerow(
                            make_snapshot_row(
                                now_wall,
                                now_perf,
                                rel_t,
                                last_values,
                                signal_time,
                            )
                        )
                        last_snapshot = rel_t

                # Periodic dashboard.
                if rel_t - last_print >= args.print_every:
                    print_dashboard(
                        rel_t,
                        last_values,
                        signal_time,
                        id_counts,
                        accepted_counts,
                        throttled_counts,
                        decode_ok,
                        decode_fail,
                        now_perf,
                        line_queue,
                        reader_stats,
                        processed_lines,
                        args.stale_ms,
                        args.show_all_signals,
                    )
                    last_print = rel_t

                # Periodic flush.
                if args.flush_every >= 0 and rel_t - last_flush >= args.flush_every:
                    if f_raw is not None:
                        f_raw.flush()
                    if f_dec is not None:
                        f_dec.flush()
                    if f_snap is not None:
                        f_snap.flush()
                    if f_sel is not None:
                        f_sel.flush()

                    last_flush = rel_t

                if rotator is not None:
                    rotator.tick(now_perf)

        except KeyboardInterrupt:
            print("\n[INFO] Ctrl+C detected. Saving logs and stopping gracefully...")
            stop_event.set()

    finally:
        # ------------------------------------------------------------
        # Shutdown sequence
        # ------------------------------------------------------------
        try:
            stop_event.set()
            if reader is not None:
                reader.join(timeout=1.0)
        except Exception:
            pass

        try:
            stop_monitor(ser)
        except Exception:
            pass

        print("[INFO] Closing log files...")

        for f in [f_raw, f_dec, f_snap, f_sel]:
            if f is not None:
                try:
                    f.flush()
                except Exception:
                    pass

                try:
                    f.close()
                except Exception:
                    pass

        # ------------------------------------------------------------
        # Summary
        # ------------------------------------------------------------
        print("[INFO] Stopped.")
        print("[INFO] Summary:")
        print(f"  raw seen frames       : {sum(id_counts.values())}")
        print(f"  accepted frames       : {sum(accepted_counts.values())}")
        print(f"  throttled frames      : {sum(throttled_counts.values())}")
        print(f"  decoded frames        : {sum(decode_ok.values())}")
        print(f"  decode fails          : {sum(decode_fail.values())}")
        print(f"  processed lines       : {processed_lines}")
        print(f"  serial bytes          : {reader_stats.get('serial_bytes', 0)}")
        print(f"  serial lines          : {reader_stats.get('serial_lines', 0)}")
        print(f"  queue dropped oldest  : {reader_stats.get('queue_dropped_oldest', 0)}")
        print(f"  ATMA restarts         : {reader_stats.get('atma_restarts', 0)}")
        print(f"  reader errors         : {reader_stats.get('reader_errors', 0)}")

        if reader_stats.get("last_reader_error"):
            print(f"  last reader error     : {reader_stats.get('last_reader_error')}")

        if reader_stats.get("last_restart_reason"):
            print(f"  last restart reason   : {reader_stats.get('last_restart_reason')}")

        if reader_stats.get("last_restart_error"):
            print(f"  last restart error    : {reader_stats.get('last_restart_error')}")

        print("\n[INFO] Top raw seen IDs:")
        for can_id, cnt in id_counts.most_common(20):
            name = id_to_msg[can_id].name if can_id in id_to_msg else "-"
            print(f"  0x{can_id:<4X} {cnt:<8} {name}")

        print("\n[INFO] Top accepted IDs:")
        for can_id, cnt in accepted_counts.most_common(20):
            name = id_to_msg[can_id].name if can_id in id_to_msg else "-"
            print(f"  0x{can_id:<4X} {cnt:<8} {name}")

        print("\n[INFO] Top throttled IDs:")
        for can_id, cnt in throttled_counts.most_common(20):
            name = id_to_msg[can_id].name if can_id in id_to_msg else "-"
            print(f"  0x{can_id:<4X} {cnt:<8} {name}")

        print("\n[INFO] Top decoded IDs:")
        for can_id, cnt in decode_ok.most_common(20):
            name = id_to_msg[can_id].name if can_id in id_to_msg else "-"
            print(f"  0x{can_id:<4X} {cnt:<8} {name}")

        if decode_fail:
            print("\n[WARN] Decode failures:")

            for can_id, cnt in decode_fail.most_common(20):
                name = id_to_msg[can_id].name if can_id in id_to_msg else "-"
                print(
                    f"  0x{can_id:<4X} {cnt:<8} "
                    f"{name} reason={fail_reason.get(can_id)}"
                )

        print("\n[INFO] Saved:")
        if not args.no_raw_log:
            print(f"  raw     : {raw_path}")

        if not args.no_decoded_log:
            print(f"  decoded : {decoded_path}")

        if not args.no_snapshot_log:
            print(f"  snapshot: {snapshot_path}")

        if not args.no_selected_log:
            print(f"  selected: {selected_path}")

        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()