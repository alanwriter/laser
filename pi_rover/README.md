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
The Nano keeps its low-level safety role and runs the existing calibrated wheel
velocity controller: PID, static-friction feed-forward, acceleration limit,
encoder measurement, PWM and expiry stop. The Pi is the single USB owner and
is responsible only for the path planner.

This command runs the default 1.5 m route with three full sine periods and a
gentle ±50 mm lateral amplitude.  Use `--amplitude-mm` when an experiment
requires another amplitude:

```text
y = 50 × sin(6πx / 1500),  x = 0…1500 mm
```

```bash
# It calibrates and checks encoders in this same USB session, asks for
# CALIBRATE, MOTOR, and WAVE-3 confirmations, then sends 180 ms wheel-speed
# setpoints to the Nano's existing controller. This next-step tuning raises
# nominal speed from 45 to 55 mm/s.
python3 leader_formation.py --port <PORT> wave --unlock --speed-mm-s 55
```

The initial tuning is deliberately conservative (45 mm/s, 10 Hz Pi replanning,
180 ms setpoint expiry). At the endpoint the default is an immediate Leader
`stopped` broadcast, so each Follower stops rather than continuing to chase a
stationary target; `--formation-settle-s` explicitly enables endpoint
convergence experiments. Clear at least a 2.0 m × 1.2 m lane. `Ctrl-C`, USB
loss, bad status, fault, finish-corridor overrun, or the 180 s deadline causes
a best-effort `STOP`.

Every wave run writes a flush-on-every-row CSV under `~/pi_rover/logs/`, for
example `wave_20261001T095440Z.csv`. It records the planned wave parameters,
Nano pose/encoder/tps/PWM/fault fields, Pi path errors, and commanded left and
right wheel speeds. Pass `--log /path/to/run.csv` to choose a specific file.
This is the file to retain and share for tuning discussion; terminal output is
only a live monitor.

It is normal for this mathematical sine wave to start and finish with a
non-zero tangent (about 51.5°): it returns to the original lateral line but
does not promise the original final heading. The terminal prints measured
`x`, `y`, target `y`, heading error, and each wheel-speed setpoint for live
monitoring.

Use `--broadcast HOST:PORT` to publish the measured leader pose while it runs.
The option may be repeated, which is preferred for the two physical followers:
`--broadcast 192.168.1.199:5005 --broadcast 192.168.1.175:5005`.  A multicast
destination such as `239.42.0.1:5005` remains supported for a network that
reliably forwards multicast.

## Leader part of the formation algorithm

The Leader does not try to steer every follower. Its job is to execute a safe
firmware path and publish an authoritative, measured reference frame:

```text
Pi path / formation planner
       ↓  (bounded IO,VELOCITY,<left_mm_s>,<right_mm_s>,<expiry>)
Nano trained wheel-speed controller + IMU/encoder odometry
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
  --broadcast 192.168.1.199:5005 --broadcast 192.168.1.175:5005
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

The earlier F1 experiment used a fixed `(d_x, d_y)` in the current Leader body
frame and calculated its target from the measured leader state:

```text
theta = heading_deg × π / 180
x_i* = x_leader + cos(theta) d_x - sin(theta) d_y
y_i* = y_leader + sin(theta) d_x + cos(theta) d_y
theta_i* = theta
```

This moving target is retained as the `FormationTracker` legacy implementation
for replaying the A/B experiment. In normal-follow `0.1.0`, both followers
instead use `PathFormationTracker`: their desired progress is `s_L−400 mm`,
their routes are translated `+400/-400 mm` in the Leader's initial frame, and
their desired headings are their own route tangents. Each follower controls
its own motors locally; Leader packets never contain PWM commands.

Firmware `PATH 1` and `PATH 2` remain useful built-in diagnostics. For new
paths, Pi computes a high-level steering correction and repeatedly sends
short wheel-speed setpoints. The Nano's existing controller—not Pi—converts
these into PWM and retains the mandatory auto-stop timeout.

## Follower 1 formation controller

`follower_formation.py` is the only process allowed to open F1's Nano USB
port. It accepts only a versioned `leader_state` UDP pose packet from the
explicit `--leader-host`; it never accepts PWM over the network. It maps F1's
locally reset odometry frame into the shared experiment frame, then sends
short `VELOCITY` targets to F1's calibrated Nano controller.

Align both cars with the same heading and place F1 exactly
400 mm rearward and 400 mm to Leader's left. The default F1 target and origin
are both `(-400 mm, +400 mm)` in the Leader's initial frame: a left-rear,
45-degree formation with 565.7 mm Leader-to-F1 separation. Keep a clear 2 m
by 1.5 m lane, physical motor-power cutoffs, and two SSH terminals.

```bash
# F1 terminal: stationary gyro calibration, short motor/encoder preflight,
# then FOLLOWER-1 at the final start pose. See RUNBOOK for the selected
# on-ground preflight procedure.
cd ~/pi_rover
python3 follower_formation.py \
  --port /dev/serial/by-id/<F1-Nano裝置名稱> \
  --leader-host 192.168.1.166 \
  --listen 0.0.0.0:5005 \
  --unlock

# Leader terminal: send the measured state directly to the two follower Pis.
# The first formation trial is deliberately a slow, shallow single wave.
cd ~/pi_rover
python3 leader_formation.py \
  --port /dev/serial/by-id/<Leader-Nano裝置名稱> \
  wave --unlock --length-mm 1000 --amplitude-mm 50 --cycles 1 \
  --speed-mm-s 25 --broadcast 192.168.1.199:5005 \
  --broadcast 192.168.1.175:5005
```

Place both rovers first. Type `FOLLOWER-1` only after F1 is placed at its start
position; this resets F1 odometry at the declared formation pose. Then type
`WAVE-3` after Leader is at the route origin; this resets Leader odometry just
before broadcast and motion. F1 begins only after three fresh Leader frames whose
IMU/encoder/fault checks pass and whose mode is `velocity` or `path`.

F1 follows the left translated route. Along-route progress error adjusts speed;
cross-track and tangent-heading errors adjust steering, with curvature
feed-forward and a bend-speed limit. It neither reverses nor pivots on its own.
A packet older than 0.35 s, missing/malformed path, stopped/faulted Leader,
F1 fault, serial loss, Ctrl-C, normal process exit, Leader distance below
150 mm or above 1200 mm, or target error above 800 mm sends F1 `STOP` and
requires a new arm. `python3 follower_formation.py --version` reports the
normal-follow software version.

Changing laboratories or switching to the vehicles' own Wi-Fi needs no code
change: join all Pi devices to that same local network, then replace only
`--leader-host` with Leader's new local address. The multicast endpoint may
stay `239.42.0.1:5005`; it is not a public Internet service.

## Follower 2 formation controller

`follower2_formation.py` uses the same path controller and safety contract as
F1. F2 starts at `(-400, -400)` mm in the
Leader's initial frame. Its route is the Leader's planned eased-sine route
translated 400 mm right; its reference progress is 400 mm behind Leader in
path arc length. F2 steers toward its own path tangent and slows where
curvature exceeds its yaw-rate or wheel-speed capacity. Align F2's heading to Leader before
arming it, then place it at the right-rear start position. RESET occurs only
after placement.

```bash
# F2 terminal. This stays stopped through calibration, wheel test and arming
# until it receives three fresh, safe moving Leader multicast packets.
cd ~/pi_rover
python3 follower2_formation.py \
  --port /dev/serial/by-id/<F2-Nano裝置名稱> \
  --leader-host 192.168.1.166 \
  --listen 0.0.0.0:5005 \
  --unlock
```

Type `FOLLOWER-2` only after F2 is physically in the right-rear position.
Both followers require a valid `eased_sine_v1` path in the Leader's `wave`
broadcasts; without it they stay stopped. The firmware `leader 1/2` PATH modes
do not publish this path and are not supported by normal-follow `0.1.0`.
Leader sends one direct UDP datagram to each Follower Pi; each process opens
only its own Nano USB port and writes a separate `follower2_<UTC timestamp>.csv`
log.

## Safety contract

- USB open waits two seconds; USB loss, protocol error, Ctrl-C, a fault, and
  process exit trigger a best-effort `IO,<seq>,STOP`.
- `PATH` is blocked until IMU, encoder and fault preflight passes.
- `TELEMETRY` is disabled in cleanup after leader mode ends.
- Always keep a physical way to remove motor power; software STOP is not a
  substitute for it.
