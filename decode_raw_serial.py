import argparse
import csv
import re
import sys
from pathlib import Path

import cantools


HEX_PAIR_RE = re.compile(r"^[0-9A-Fa-f]{2}$")
HEX_ID_RE = re.compile(r"^[0-9A-Fa-f]{1,8}$")


def parse_atma_line(line: str):
    """
    Parse common OBDLink/ELM ATMA text lines.

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
            return None

        tokens.append(tok)

    if len(tokens) == 1:
        lone = tokens[0]
        if len(lone) >= 5:
            can_id_token = lone[:3]
            data_token = lone[3:]
            if HEX_ID_RE.match(can_id_token) and len(data_token) % 2 == 0:
                data_bytes = bytes.fromhex(data_token)
                return int(can_id_token, 16), data_bytes[:8]
        return None

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
            for i in range(0, len(tok), 2):
                expanded.append(tok[i:i + 2])
        else:
            expanded.append(tok)
    data_tokens = expanded

    # Optional DLC token format: "130 8 00 00 00 00 00 00 04 2E"
    if len(data_tokens) >= 2:
        try:
            if len(data_tokens[0]) == 1 and data_tokens[0].isdigit():
                data_tokens = data_tokens[1:]
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


def load_dbc(dbc_path: Path):
    try:
        return cantools.database.load_file(str(dbc_path), strict=False)
    except Exception:
        # Remove malformed signal comments if present.
        sanitized_lines = []
        for line in dbc_path.read_text(encoding="utf-8").splitlines():
            if re.match(r"^\s*CM_\s+SG_\s+\d+\s+\"", line):
                continue
            sanitized_lines.append(line)
        return cantools.database.load_string("\n".join(sanitized_lines), database_format="dbc", strict=False)


def iter_lines_with_ts(raw_lines_path: Path):
    with raw_lines_path.open("r", encoding="utf-8", newline="") as f_in:
        reader = csv.reader(f_in)
        header = next(reader, None)
        for row in reader:
            if len(row) < 4:
                continue
            yield row[0], row[1], row[2], row[3]


def iter_lines_from_bytes(input_path: Path):
    with input_path.open("rb") as f_in:
        buf = ""
        while True:
            chunk = f_in.read(65536)
            if not chunk:
                break

            text = chunk.decode("ascii", errors="ignore")
            text = text.replace("\r", "\n")
            buf += text

            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                yield None, None, None, line


def decode_stream(
    input_path: Path,
    raw_out: Path,
    decoded_out: Path,
    dbc_path: Path | None,
    raw_lines_path: Path | None,
) -> int:
    db = None
    id_to_msg = {}
    if dbc_path is not None:
        if not dbc_path.exists():
            print(f"[ERROR] DBC not found: {dbc_path}")
            return 1
        db = load_dbc(dbc_path)
        id_to_msg = {m.frame_id: m for m in db.messages}
        print(f"[INFO] Loaded DBC: {dbc_path} ({len(db.messages)} messages)")

    with raw_out.open("w", newline="", encoding="utf-8") as f_raw, \
            decoded_out.open("w", newline="", encoding="utf-8") as f_dec:
        raw_writer = csv.writer(f_raw)
        dec_writer = csv.writer(f_dec)

        raw_writer.writerow(["time_wall", "time_perf", "rel_time", "line_index", "can_id", "dlc", "data_hex"])
        dec_writer.writerow(["time_wall", "time_perf", "rel_time", "line_index", "can_id", "message", "signal", "value"])

        line_index = 0
        raw_count = 0
        dec_count = 0

        if raw_lines_path is not None:
            line_iter = iter_lines_with_ts(raw_lines_path)
        else:
            line_iter = iter_lines_from_bytes(input_path)

        for time_wall, time_perf, rel_time, line in line_iter:
            line_index += 1
            parsed = parse_atma_line(line)
            if parsed is None:
                continue

            if parsed[0] in ("__PROMPT__", "__ADAPTER_STATUS__"):
                continue

            can_id, data = parsed
            raw_writer.writerow([
                time_wall or "",
                time_perf or "",
                rel_time or "",
                line_index,
                f"0x{can_id:X}",
                len(data),
                data.hex().upper(),
            ])
            raw_count += 1

            if db is None:
                continue

            msg = id_to_msg.get(can_id)
            if msg is None:
                continue

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
            except Exception:
                continue

            for sig, val in decoded.items():
                dec_writer.writerow([
                    time_wall or "",
                    time_perf or "",
                    rel_time or "",
                    line_index,
                    f"0x{can_id:X}",
                    msg.name,
                    sig,
                    val,
                ])
                dec_count += 1

        print(f"[INFO] Raw frames: {raw_count}")
        print(f"[INFO] Decoded rows: {dec_count}")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Decode raw OBDLink serial capture to CSV.")
    parser.add_argument("--input", required=True, type=Path, help="Path to raw_serial_*.bin")
    parser.add_argument("--dbc", type=Path, default=None, help="DBC file path for decoding")
    parser.add_argument("--raw-out", type=Path, default=Path("decoded_raw.csv"), help="Raw CSV output path")
    parser.add_argument("--decoded-out", type=Path, default=Path("decoded_signals.csv"), help="Decoded CSV output path")
    parser.add_argument("--raw-lines", type=Path, default=None, help="Optional raw_lines_*.csv with timestamps")
    args = parser.parse_args()

    if not args.input.exists():
        print(f"[ERROR] Input not found: {args.input}")
        return 1

    if args.raw_lines is not None and not args.raw_lines.exists():
        print(f"[ERROR] Raw lines file not found: {args.raw_lines}")
        return 1

    return decode_stream(args.input, args.raw_out, args.decoded_out, args.dbc, args.raw_lines)


if __name__ == "__main__":
    sys.exit(main())
