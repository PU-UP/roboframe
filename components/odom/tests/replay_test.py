#!/usr/bin/env python3
"""Check the receiver's exit status and dataset result; gzip errors must fail."""
import argparse
import gzip
from pathlib import Path
import subprocess
import sys
import tempfile

EXPECTED = (
    "accepted=26915 first_us=976052857337284 x=0.000000 y=0.000000 yaw=-0.002458 "
    "last_us=976055548625127 x=-50.883999 y=-35.825001 yaw=2.538102"
)


def replay(receiver, path):
    # This test uses one fixed dataset; read validates its gzip trailer before playback.
    with gzip.open(path, "rb") as stream:
        payload = stream.read()
    result = subprocess.run([receiver], input=payload, capture_output=True, timeout=30)
    if result.returncode:
        raise RuntimeError(f"receiver exited {result.returncode}: {result.stderr.decode(errors='replace')}")
    output = result.stdout.decode().strip()
    if output != EXPECTED:
        raise RuntimeError(f"unexpected receiver output: {output!r}")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("receiver")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--check-corrupt", action="store_true")
    args = parser.parse_args()
    if not args.check_corrupt:
        print(replay(args.receiver, args.dataset))
        return

    with tempfile.TemporaryDirectory(prefix="odom-corrupt-") as directory:
        damaged = bytearray(args.dataset.read_bytes())
        damaged[-8] ^= 1  # Break CRC while preserving every decompressed record.
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
