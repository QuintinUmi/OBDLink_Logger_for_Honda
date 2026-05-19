import argparse
import csv
from pathlib import Path


def sanitize(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in name)


def pick_timestamp(row: dict) -> str:
    for key in ("timestamp", "time_wall", "rel_time", "time_perf"):
        value = row.get(key, "")
        if str(value).strip() != "":
            return value
    return ""


def split_by_signal(input_path: Path, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    writers = {}
    files = {}

    def get_writer(key: str):
        if key not in writers:
            safe = sanitize(key)
            out_path = output_dir / f"{safe}.csv"
            f = out_path.open("w", newline="", encoding="utf-8")
            w = csv.writer(f)
            w.writerow(["timestamp", "class_name", "value"])
            files[key] = f
            writers[key] = w
        return writers[key]

    with input_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            message = row.get("message", "").strip()
            signal = row.get("signal", "").strip()
            if not message or not signal:
                continue
            full_signal = row.get("full_signal", "").strip()
            key = full_signal or f"{message}.{signal}"
            writer = get_writer(key)
            writer.writerow([
                pick_timestamp(row),
                key,
                row.get("value", ""),
            ])

    for f in files.values():
        f.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Split decoded CAN CSV by message.signal.")
    parser.add_argument("input_csv", type=Path, help="Path to decoded.csv or can_decoded.csv")
    parser.add_argument("output_dir", type=Path, help="Directory to store per-signal CSVs")
    args = parser.parse_args()

    split_by_signal(args.input_csv, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
