"""Read-only Leader Nano boot/MPU probe; each serial open resets the Nano."""

from __future__ import annotations

import contextlib
import io
import json
import sys

from leader_formation import NanoLink


PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
CYCLES = 12
READS_PER_BOOT = 8


def main() -> int:
    results = []
    for cycle in range(1, CYCLES + 1):
        link = NanoLink(PORT)
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                link.open()
                hello = link.request("HELLO", 1)
                config = link.request("CONFIG")
                imu = [link.request("IMU") for _ in range(READS_PER_BOOT)]
            row = {
                "cycle": cycle,
                "profile": hello.fields,
                "heading_blend": config.fields[8],
                "present": [item.fields[0] for item in imu],
                "addresses": [item.fields[2] for item in imu],
                "read_failures": [item.fields[9] for item in imu],
                "consecutive_failures": [item.fields[10] for item in imu],
            }
            results.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
        finally:
            link.close()
    passed = all(
        row["profile"][0] == "1"
        and "FIRMWARE_PROFILE=L1" in row["profile"][2]
        and row["heading_blend"] == "1.00"
        and all(present == "1" for present in row["present"])
        and all(address == "104" for address in row["addresses"])
        and all(failures == "0" for failures in row["consecutive_failures"])
        for row in results
    )
    print(json.dumps({"cycles": len(results), "passed": passed}), flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
