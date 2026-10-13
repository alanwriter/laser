#!/usr/bin/env python3
"""Follower-2 right-rear formation entry point.

This intentionally reuses the F1 controller, packet validation, Nano safety
gates and CSV schema.  F2 differs only in its declared Leader-body target and
RESET origin: (-400, -400) mm, i.e. 400 mm behind and 400 mm right of Leader.
"""

from follower_formation import FOLLOWER_2, main


if __name__ == "__main__":
    raise SystemExit(main(FOLLOWER_2))
