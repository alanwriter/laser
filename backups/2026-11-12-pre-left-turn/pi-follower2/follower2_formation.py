#!/usr/bin/env python3
"""Follower-2 path-following entry point.

F2 starts at (-400, -400) mm. It follows a copy of Leader's planned route
translated 400 mm right and tracks a 400 mm gap along the route. Packet
handling, Nano commissioning, and safety gates remain shared with F1.
"""

from follower_formation import FOLLOWER_2, main


if __name__ == "__main__":
    raise SystemExit(main(FOLLOWER_2))
