#!/usr/bin/env python3
"""Compatibility entry point for the current Leader-1 USB protocol.

The former implementation used the retired single-letter protocol
(`S/I/D/P/G1`).  Leader-1 firmware requires `IO,<sequence>,<operation>`.
Keep this name so existing deployment commands run the new safe client.
"""

from leader_formation import main


if __name__ == "__main__":
    raise SystemExit(main())
