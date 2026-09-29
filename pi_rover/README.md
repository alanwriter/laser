# Raspberry Pi rover controller

`rover_control.py` drives the existing Nano firmware over USB serial. The Nano
remains responsible for motor PI control, encoder handling, odometry, MPU6050
fusion, timeouts, and emergency motor stop.

The program is intentionally an interactive command-line controller, not a
systemd service: booting the Pi must never start the motors.

## Install on the Pi

From the Mac project folder:

```bash
scp -r pi_rover alan@192.168.1.199:~/
```

Then, on the Pi:

```bash
cd ~/pi_rover
python3 rover_control.py inspect
```

The Pi currently detects the Nano as a CH340 converter. Its `by-id` path is
absent because the converter exposes no unique serial number, so the default
uses the stable `by-path` link for the USB socket currently used. If the Nano
is moved to a different Pi USB socket, run `ls -l /dev/serial/by-path/` and
pass its new path explicitly with `--port`.

## Commands

```bash
# Stops first, then checks MPU/encoder/fault status. Does not move.
python3 rover_control.py inspect

# Stops immediately. Also used automatically on Ctrl-C and normal exit.
python3 rover_control.py stop

# With the car completely stationary, calibrate the MPU6050 gyro Z bias.
python3 rover_control.py calibrate --confirm-still

# Suspended-wheel wiring/direction test: M80,80 for about 0.2 seconds.
python3 rover_control.py pulse --seconds 0.2 --pwm 80 --unlock

# Explicitly unlock, then type G1 at the confirmation prompt.
python3 rover_control.py run G1 --unlock
```

Available firmware trajectories:

| Command | Nano trajectory |
| --- | --- |
| `G1` | 500 mm straight line |
| `G2` | 400 mm × 400 mm square |
| `G3` | 350 mm L-shaped path |

`run` always sends `S`, reads `I`, `D`, and `P`, and blocks the trajectory
unless it sees `present=yes`, `calibrated=yes`, `encoder_preflight=passed`, and
`fault=none`. It then requires both `--unlock` and a typed trajectory name.
Pressing Ctrl-C makes a best-effort `S` transmission before the program exits.

`pulse` is only a short wiring/direction test. It sends equal left/right PWM,
waits for the requested duration, and sends `S`; its duration is approximate
and must not be used to target a number of wheel rotations.

Before moving, clear the route, keep the wheels off the ground for initial
tests, and retain a physical way to remove motor power. A serial disconnect is
also handled by the Nano's firmware-side safety timeout; it is not a substitute
for a physical emergency stop.

## Scope

The installed Nano firmware exposes only `G1`, `G2`, and `G3`; it has no
protocol for arbitrary waypoints, velocities, or continuous Pi-side trajectory
streaming. Supporting other paths requires adding a new, safety-reviewed Nano
firmware command rather than attempting wheel PID control from Linux.
