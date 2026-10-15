#!/usr/bin/env python3
"""Exercise asynchronous/BT mode at its process boundary, including existing session mode."""
import gzip
import subprocess
import sys

receiver, dataset = sys.argv[1:]
with gzip.open(dataset, "rb") as stream:
    payload = stream.read()
result = subprocess.run([receiver, "--mission", "1", "2"],
                        input=payload, capture_output=True, timeout=30)
if result.returncode:
    raise RuntimeError(result.stderr.decode())
lines = result.stdout.decode().splitlines()
summary = ("MISSION state=succeeded steps=2 valid=26915 latest_us=976055548625127 "
           "x=-50.883999 y=-35.825001 yaw=2.538102 distance=509.139428")
assert lines[-1] == summary, lines[-1:]
progress = [line for line in lines if line.startswith("PROGRESS ")]
assert progress and progress[-1] == "PROGRESS active=0 meters=0 target=0", progress[-3:]

pose = b"ODOM 0 0 0 0 0 0 1 host 0\n"
for data in (pose, pose + b"MISSION cancel\n"):
    result = subprocess.run([receiver, "--mission", "1", "2"],
                            input=data, capture_output=True, timeout=5)
    assert result.returncode == 2, result
    assert b"MISSION state=canceled" in result.stdout, result
result = subprocess.run([receiver, "--mission", "1", "2"],
                        input=b"ODOM nan 0 0 0 0 0 1 host 0\n", capture_output=True, timeout=5)
assert result.returncode == 1 and b"MISSION state=succeeded" not in result.stdout, result
assert result.stdout.decode().splitlines()[-1] == "PROGRESS active=0 meters=0 target=0", result
for args in (["--mission", "2", "1"], ["--mission", "nan", "2"], ["--mission", "1x", "2"]):
    result = subprocess.run([receiver, *args], input=b"", capture_output=True, timeout=5)
    assert result.returncode == 1, result
print(summary)
print("EOF/cancel, invalid goals and final inactive output verified")
