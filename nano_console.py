"""Forward direct serial commands to an existing Nano firmware.

The console does not upload or modify Nano firmware.  It sends the exact
newline-terminated command entered by the user; `160,30` is the expected
pan/tilt form for this vision project.
"""

from __future__ import annotations

import argparse
import time

import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send direct P/T commands to an Arduino Nano.")
    parser.add_argument(
        "--port",
        default="/dev/cu.usbserial-1320",
        help="Nano serial device (default: /dev/cu.usbserial-1320)",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--command", help="Send one command and exit, e.g. '160,30'.")
    return parser.parse_args()


def send_and_print(device: serial.Serial, command: str) -> None:
    # Pasted Traditional-Chinese text can contain a full-width comma/equal
    # sign.  Normalise it here; Arduino IDE Serial Monitor still needs ASCII.
    command = command.strip().replace("，", ",").replace("＝", "=")
    device.write((command + "\n").encode("ascii"))
    device.flush()
    reply = device.readline().decode("ascii", errors="replace").strip()
    print(reply or "(No reply from Nano)")


def main() -> None:
    args = parse_args()
    with serial.Serial(args.port, args.baud, timeout=1.0) as device:
        # Opening a Nano serial port normally resets it.
        time.sleep(2.0)
        device.reset_input_buffer()

        if args.command:
            send_and_print(device, args.command)
            return

        print("Enter the command accepted by your existing Nano firmware (for example 160,30). Type q to quit.")
        while True:
            try:
                command = input("Nano> ").strip()
            except EOFError:
                break
            if command.lower() in {"q", "quit", "exit"}:
                break
            if command:
                send_and_print(device, command)


if __name__ == "__main__":
    main()
