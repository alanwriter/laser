# 2026-10-13 three-rover backup

This is a read-only snapshot copied from the three Raspberry Pis at the end of
the three-rover formation-control session.

- `pi-leader/` — Leader controller, deployed Nano HEX, and 13 recorded CSV logs.
- `pi-follower1/` — Follower 1 controller, deployed Nano HEX, and 4 recorded CSV logs.
- `pi-follower2/` — Follower 2 controller and 3 recorded CSV logs.

The CSV files are intentionally stored with their corresponding controller so
the program version, deployed HEX, and raw measurement data remain together.
The three CSV files used for the final combined trajectory figure are also
available in `analysis/2026-10-13-three-rover/`.
