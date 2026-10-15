#!/usr/bin/env python3
"""Replay intel ODOM through the product and check session control."""
import argparse
import gzip
import math
from pathlib import Path
import subprocess
import sys
import tempfile

EXPECTED_PREFIX = (
    "valid=26915 latest_us=976055548625127 "
    "x=-50.883999 y=-35.825001 yaw=2.538102"
)
SESSION_INPUT = """ODOM 0.000000 0.000000 0.000000 0 0 0 1.000000 nohost 0.000000
FLASER skip
# comment
PARAM skip
ODOM 3.000000 4.000000 0.500000 0 0 0 2.000000 nohost 0.000000
SESSION stop
SESSION restart
ODOM 3.000000 4.000000 0.500000 0 0 0 3.000000 nohost 0.000000
ODOM 3.000000 5.000000 0.250000 0 0 0 4.000000 nohost 0.000000
"""
SESSION_EXPECTED = (
    "valid=2 latest_us=2000000 x=3.000000 y=4.000000 yaw=0.500000 distance=5.000000\n"
    "valid=2 latest_us=4000000 x=3.000000 y=5.000000 yaw=0.250000 distance=1.000000"
)


def trajectory(payload):
    points = []
    for line in payload.decode().splitlines():
        if not line.startswith("ODOM "):
            continue
        fields = line.split()
        points.append((float(fields[1]), float(fields[2])))
    distance = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        distance += math.hypot(x1 - x0, y1 - y0)
    return len(points), distance


def replay(receiver, path):
    with gzip.open(path, "rb") as stream:
        payload = stream.read()
    count, distance = trajectory(payload)
    if count != 26915:
        raise RuntimeError(f"dataset ODOM count changed: {count}")
    expected = f"{EXPECTED_PREFIX} distance={distance:.6f}"
    result = subprocess.run([receiver], input=payload, capture_output=True, timeout=30)
    if result.returncode:
        raise RuntimeError(
            f"receiver exited {result.returncode}: {result.stderr.decode(errors='replace')}")
    output = result.stdout.decode().strip()
    if not output.startswith(EXPECTED_PREFIX + " "):
        raise RuntimeError(f"session count or pose regressed: {output!r}")
    if output != expected:
        raise RuntimeError(f"unexpected receiver output: {output!r}")
    return output


def session_control(receiver):
    stopped, remaining = SESSION_INPUT.split("SESSION restart\n", 1)
    first_summary = SESSION_EXPECTED.splitlines()[0]
    for prefix, summary in [
        (stopped, first_summary),
        ("SESSION start\nSESSION stop\n", "valid=0 distance=0.000000"),
    ]:
        result = subprocess.run(
            [receiver], input=(prefix + remaining), capture_output=True, text=True, timeout=10)
        if result.returncode != 1 or "session is not running" not in result.stderr:
            raise RuntimeError(f"ODOM after stop was not rejected: {result}")
        # Error exit may occur before the writer flushes the already stopped session.
        if result.stdout.strip() not in ("", summary):
            raise RuntimeError(f"ODOM after stop produced a new session: {result.stdout!r}")

    for payload, expected in [
        (SESSION_INPUT, SESSION_EXPECTED),
        (SESSION_INPUT.replace("SESSION restart", "SESSION start"), SESSION_EXPECTED),
        (SESSION_INPUT.replace("SESSION stop\nSESSION restart", "SESSION restart"), SESSION_EXPECTED),
        (stopped, first_summary),
        ("SESSION start\nSESSION stop\n", "valid=0 distance=0.000000"),
    ]:
        result = subprocess.run(
            [receiver], input=payload, capture_output=True, text=True, timeout=10)
        if result.returncode or result.stdout.strip() != expected:
            raise RuntimeError(f"unexpected session control result: {result}")
    return "stop rejects ODOM; explicit start/restart and EOF summaries verified"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("receiver")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--check-corrupt", action="store_true")
    parser.add_argument("--check-session", action="store_true")
    args = parser.parse_args()
    if args.check_session:
        print(session_control(args.receiver))
        return
    if not args.check_corrupt:
        print(replay(args.receiver, args.dataset))
        return

    with tempfile.TemporaryDirectory(prefix="odom-session-corrupt-") as directory:
        damaged = bytearray(args.dataset.read_bytes())
        damaged[-8] ^= 1
        path = Path(directory) / "bad-crc.gz"
        path.write_bytes(damaged)
        result = subprocess.run(
            [sys.executable, __file__, args.receiver, str(path)],
            capture_output=True, text=True, timeout=40)
        if result.returncode != 1 or "CRC check failed" not in result.stderr:
            raise RuntimeError(f"corrupt gzip was not rejected correctly: {result}")
    print("corrupt gzip rejected with exit 1")


if __name__ == "__main__":
    try:
        main()
    except (OSError, EOFError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(error, file=sys.stderr)
        sys.exit(1)
