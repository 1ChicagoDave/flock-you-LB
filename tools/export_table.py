#!/usr/bin/env python3
"""Pull the stored detection table off the device over USB serial.

Sends the firmware's one-key export command ('d' = CSV, 'j' = JSON) and
captures the reply to a timestamped file under exports/. Reusable in the field.

Usage:
    python tools/export_table.py [--port COM4] [--fmt csv|json] [--baud 115200]
"""
import argparse
import datetime
import os
import sys
import time

try:
    import serial  # pyserial (bundled with PlatformIO's esptool)
except ImportError:
    sys.exit("pyserial not found; run with PlatformIO's penv python")

CMD = {"csv": b"d", "json": b"j"}
# CSV export starts with this header line; both formats end with this marker.
CSV_HEADER = "mac,method,rssi,channel,count"
END_MARK = "[flockyou] dumped"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--fmt", choices=("csv", "json"), default="csv")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--boot-wait", type=float, default=5.0,
                    help="seconds to let the device boot/flush its banner first")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.3)
    # Let the board finish booting (LED self-test + SPIFFS reload) and drain the
    # startup banner so it doesn't end up in our capture.
    time.sleep(args.boot_wait)
    ser.reset_input_buffer()

    ser.write(CMD[args.fmt])
    ser.flush()

    # Read until the device goes quiet (no data for ~1.2s) or a hard cap.
    chunks, last_rx, deadline = [], time.time(), time.time() + 15
    while time.time() < deadline:
        data = ser.read(4096)
        if data:
            chunks.append(data)
            last_rx = time.time()
        elif time.time() - last_rx > 1.2:
            break
    ser.close()

    text = b"".join(chunks).decode("utf-8", "replace")
    lines = text.splitlines()

    # Slice out just the export block (drop any stray banner/detection lines).
    start = 0
    if args.fmt == "csv":
        for i, ln in enumerate(lines):
            if ln.startswith(CSV_HEADER):
                start = i
                break
    body, count = [], "?"
    for ln in lines[start:]:
        if ln.startswith(END_MARK):
            count = ln.split("dumped", 1)[1].strip().split()[0]
            break
        body.append(ln)

    os.makedirs("exports", exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    ext = "csv" if args.fmt == "csv" else "jsonl"
    path = os.path.join("exports", f"flock-table-{stamp}.{ext}")
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("\n".join(body).rstrip("\n") + "\n")

    print(f"exported {count} detections -> {path} ({len(body)} lines)")


if __name__ == "__main__":
    main()
