# Leader-1 Raspberry Pi controller

`leader_formation.py` is the Raspberry Pi owner of the Leader-1 Nano USB
connection. It implements the L1 `IO,<sequence>,<OPERATION>` protocol at
115200 baud. The Nano remains responsible for motor control, encoder odometry,
IMU integration and its own motor timeout.

`rover_control.py` is retained only as a compatibility entry point and runs the
same L1 controller. Do **not** use scripts written for the earlier single-letter
`S/I/D/P/G1` protocol with this firmware.

## Connect the Leader

1. Connect the Pi's USB host port to the Leader Nano's USB port. Do not use
   Nano D0/D1 directly.
2. On the Pi, identify the stable device link:

   ```bash
   ls -l /dev/serial/by-id/
   ```

   Use the resulting full path with `--port`. If the USB converter has no
   unique serial number and `by-id` is absent, use the particular physical USB
   socket's path from `ls -l /dev/serial/by-path/`; never use `/dev/ttyUSB0`.
3. Install the one runtime dependency and run the non-moving handshake:

   ```bash
   cd ~/pi_rover
   python3 -m pip install --user pyserial
   python3 leader_formation.py \
     --port /dev/serial/by-id/<Nano裝置名稱> status
   ```

Opening the USB serial port can reset the Nano. The client waits two seconds,
then always performs `HELLO → STOP → IMU → ENCODER → STATUS`. It allows only
one process to own the port.

`status` must report all of the following before a path can run:

- `imu_present=1`
- `imu_calibrated=1`
- `encoder_preflight=1`
- `fault_code=0`

If calibration is required, keep the rover motionless and run:

```bash
python3 leader_formation.py --port /dev/serial/by-id/<Nano裝置名稱> \
  calibrate --confirm-still
```

This opens the port, stops first, then additionally requires typing
`CALIBRATE` before sending the command.

## Safe operations

```bash
# These never command motion.
python3 leader_formation.py --port <PORT> config
python3 leader_formation.py --port <PORT> imu
python3 leader_formation.py --port <PORT> encoder
python3 leader_formation.py --port <PORT> reset

# Immediate STOP, including after Ctrl-C, a fault, serial failure, and normal exit.
python3 leader_formation.py --port <PORT> stop

# Timed manual test. Nano enforces -165..165 PWM and 50..1200 ms.
python3 leader_formation.py --port <PORT> motor 80 80 500 --unlock
```

`motor` requires `--unlock` and a typed `MOTOR` confirmation. Pi boot never
sends `MOTOR` or `PATH`.

## Full-vehicle commissioning

Opening a Nano USB serial port resets the Nano, including its volatile gyro
calibration and encoder counts. Therefore do **not** calibrate in one command,
test wheels in another, then expect a third command to retain that state.
Use one commissioning session instead:

```bash
# Keep the car still for calibration, then lift both wheels before MOTOR.
python3 leader_formation.py --port <PORT> commission --unlock
```

It requires typing `CALIBRATE`, then `MOTOR`; it sends equal PWM `80` for
`800 ms`, stops, reads both encoder A/B phases, and reports `COMMISSION
PASSED` only when `imu_present=1`, `imu_calibrated=1`,
`encoder_preflight=1`, and `fault_code=0` coexist in the same USB session.

## Pi-planned wave test

Arbitrary experimental trajectories belong on the Pi, not in Nano firmware.
The Nano keeps its low-level safety role: it accepts only bounded `MOTOR` PWM
commands (each expires within 1200 ms), measures pose, and stops on `STOP` or a
fault. The Pi is the single USB owner and is responsible for the path planner.

This command runs the requested 3 m route with three full sine periods and
±200 mm lateral amplitude:

```text
y = 200 × sin(6πx / 3000),  x = 0…3000 mm
```

```bash
# It calibrates and checks encoders in this same USB session, asks for
# CALIBRATE, MOTOR, and WAVE-3 confirmations, then sends 320 ms PWM pulses.
python3 leader_formation.py --port <PORT> wave --unlock
```

The initial tuning is deliberately conservative (`PWM 70`, 4 Hz replanning,
PWM range 30–120). Clear at least a 3.5 m × 1.2 m lane. `Ctrl-C`, USB loss,
bad status, fault, or the 150 s deadline causes a best-effort `STOP`.

It is normal for this mathematical sine wave to start and finish with a
non-zero tangent (about 51.5°): it returns to the original lateral line but
does not promise the original final heading. The terminal prints measured
`x`, `y`, target `y`, heading error, and each PWM command for tuning.

Use `--broadcast HOST:PORT` to publish the measured leader pose while it runs,
for example `--broadcast 239.42.0.1:5005`.

## Leader part of the formation algorithm

The Leader does not try to steer every follower. Its job is to execute a safe
firmware path and publish an authoritative, measured reference frame:

```text
Pi path / formation planner
       ↓  (bounded IO,MOTOR,<left>,<right>,<expiry>)
Nano motor actuation + IMU/encoder odometry
       ↓  (IO,STATUS,...)
Pi leader publisher
       ↓  JSON UDP or stdout
Follower reference generator
       ↓
each follower's local safety checks and motor controller
```

Start a leader path only after the preflight checks, explicit `--unlock`, and
typed confirmations. The command commissions the car in the same USB session
before it issues `PATH`, so opening the port cannot erase the required state:

```bash
# PATH 1 is 500 mm straight; PATH 2 is 700 mm square.
# Broadcast measured leader state to the follower network every 500 ms.
python3 leader_formation.py --port <PORT> leader 1 --unlock \
  --broadcast 239.42.0.1:5005
```

`--telemetry-ms` accepts `100–2000` ms; the default `500` ms is appropriate
for the initial full-vehicle test.

The broadcast is UTF-8 JSON, one UDP datagram per observed pose:

```json
{"type":"leader_state","version":1,"monotonic_s":123.456,"status":{"x_mm":0.0,"y_mm":0.0,"heading_deg":0.0}}
```

The complete `status` object also contains mode, wheel counts/speeds/PWM,
fault and preflight flags, and the active path/step. A follower must reject
stale frames, a nonzero `fault_code`, or a non-ready leader; it must stop itself
when frames time out. UDP is intentionally optional: without `--broadcast`,
the exact same frames are printed to stdout for logging and test.

For follower *i* with a desired fixed formation offset `(d_x, d_y)` expressed
in the Leader body frame, calculate the target in world coordinates from the
measured leader state:

```text
theta = heading_deg × π / 180
x_i* = x_leader + cos(theta) d_x - sin(theta) d_y
y_i* = y_leader + sin(theta) d_x + cos(theta) d_y
theta_i* = theta
```

This is a leader–follower rigid-formation reference generator: it compensates
for the Leader's actual pose drift before commands reach the followers. Each
follower should perform its own local pose control and never forward raw motor
commands received over UDP.

Firmware `PATH 1` and `PATH 2` remain useful built-in diagnostics. For new
paths, the Pi does not bypass Nano safety or claim Nano offers a velocity API:
it computes a high-level steering correction and repeatedly sends short raw
PWM commands, each with Nano's mandatory auto-stop timeout.

## Safety contract

- USB open waits two seconds; USB loss, protocol error, Ctrl-C, a fault, and
  process exit trigger a best-effort `IO,<seq>,STOP`.
- `PATH` is blocked until IMU, encoder and fault preflight passes.
- `TELEMETRY` is disabled in cleanup after leader mode ends.
- Always keep a physical way to remove motor power; software STOP is not a
  substitute for it.
