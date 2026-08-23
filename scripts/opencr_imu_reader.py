#!/usr/bin/env python3
"""Read CSV rows from the test-only OpenCR IMU stream sketch."""

import argparse
import csv
import os
import subprocess
import sys
import time


DEFAULT_PORT = "/dev/opencr"
DEFAULT_BAUD = 115200


def configure_serial_port(port: str, baud: int) -> None:
    subprocess.run(
        ["stty", "-F", port, str(baud), "raw", "-echo"],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def wait_for_header(serial_file, timeout_sec: float) -> list[str]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        line = serial_file.readline()
        if not line:
            continue

        text = line.decode("utf-8", errors="replace").strip()
        if not text:
            continue
        if text.startswith("error,"):
            raise RuntimeError(text)
        if text.startswith("time_ms,"):
            return next(csv.reader([text]))

    raise TimeoutError("IMU CSV header was not received.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--rows", type=int, default=20)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    args = parser.parse_args()

    if not os.path.exists(args.port):
        print(f"Serial port not found: {args.port}", file=sys.stderr)
        return 2

    try:
        configure_serial_port(args.port, args.baud)
        with open(args.port, "rb", buffering=0) as serial_file:
            header = wait_for_header(serial_file, args.timeout_sec)
            print(",".join(header))

            for _ in range(max(1, args.rows)):
                line = serial_file.readline()
                if not line:
                    continue
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    print(text)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, subprocess.CalledProcessError, TimeoutError) as exc:
        print(f"Failed to read OpenCR IMU stream: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
