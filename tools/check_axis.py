#!/usr/bin/env python3
"""Verify control_demo's SDK axis remap from the fake-robot wire log.

The web/ROS order is [forward, yaw, strafe, head]; the SDK/robot order is
[forward, strafe, yaw, head]. With max_speed_scale=0.5, sending yaw=0.5 must
produce wire [0,0,-0.25,0] and strafe=0.5 must produce [-0.25,0,0,0]
(ZsibotPacker negates all axes and swaps the first two).

Run while control_demo launch + tools/fake_robot.py are active:

    python3 tools/check_axis.py --log /tmp/cd_fake.jsonl
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def send(base: str, seq: int, yaw: float, strafe: float) -> None:
    payload = {
        "forward": 0.0,
        "yaw": yaw,
        "strafe": strafe,
        "head": 0.0,
        "seq": seq,
    }
    request = urllib.request.Request(
        base.rstrip("/") + "/api/joy",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=3) as response:
        if response.status != 200:
            raise RuntimeError(f"joy HTTP {response.status}")


def vectors(path: str) -> set[tuple[float, float, float, float]]:
    found: set[tuple[float, float, float, float]] = set()
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            try:
                packet = json.loads(line).get("packet", {})
            except ValueError:
                continue
            if packet.get("type") != "remote":
                continue
            joystick = packet.get("joystick") or []
            if len(joystick) >= 4 and any(abs(float(value)) > 0.01 for value in joystick):
                found.add(tuple(round(float(value), 3) for value in joystick[:4]))
    return found


def close(vector: tuple[float, float, float, float], expected: tuple[float, float, float, float]) -> bool:
    return all(abs(a - b) <= 0.03 for a, b in zip(vector, expected))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--log", default="/tmp/cd_fake.jsonl")
    args = parser.parse_args()

    send(args.base_url, 1, 0.5, 0.0)
    time.sleep(0.35)
    send(args.base_url, 2, 0.0, 0.0)
    time.sleep(0.3)
    send(args.base_url, 3, 0.0, 0.5)
    time.sleep(0.35)
    send(args.base_url, 4, 0.0, 0.0)
    time.sleep(0.3)

    seen = vectors(args.log)
    yaw_vector = (0.0, 0.0, -0.25, 0.0)
    strafe_vector = (-0.25, 0.0, 0.0, 0.0)
    yaw_ok = any(close(vector, yaw_vector) for vector in seen)
    strafe_ok = any(close(vector, strafe_vector) for vector in seen)
    print(("PASS  " if yaw_ok else "FAIL  ") + "yaw -> wire[2] (-0.25)")
    print(("PASS  " if strafe_ok else "FAIL  ") + "strafe -> wire[0] (-0.25)")
    print("nonzero wire vectors:", sorted(seen)[:16])
    return 0 if yaw_ok and strafe_ok else 1


if __name__ == "__main__":
    sys.exit(main())
