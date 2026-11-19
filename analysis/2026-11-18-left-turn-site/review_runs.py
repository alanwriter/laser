#!/usr/bin/env python3
"""Rebuild the two 11/18 run reviews from CSV and local, untracked MOV files.

Video panels are independent visual evidence. They are deliberately not
projected into the odometry coordinate system: the handheld, oblique camera
has no measured ground control points or MPU-top fiducials.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "pi_rover"))
from turn_path import TurnPath, TurnPathReference  # noqa: E402

RUNS = {
    "real_run_122341Z": ("IMG_8212.MOV", 200, 30),
    "real_run_123302Z": ("IMG_8213.MOV", 400, 38),
}
COLORS = {"Leader": "#374151", "F1": "#2563eb", "F2": "#ea580c"}
START = {"Leader": (0, 0), "F1": (-400, 400), "F2": (-400, -400)}
BODY = np.array([[100, 75], [100, -75], [-150, -75], [-150, 75], [100, 75]])
FIELDS = {
    "Leader": ("status_", "control"),
    "F1": ("follower_", "control_moving"),
    "F2": ("follower_", "control_moving"),
}
FEEDFORWARD = {
    "Leader": ((30, .050), (35, 1 / 30)),
    "F1": ((22.4, .0126), (29.9, .0143)),
    "F2": ((18, .0200), (20.7, .0147)),
}


def number(row, key):
    value = row.get(key, "")
    return float(value) if value not in (None, "") else math.nan


def col(rows, key):
    return np.array([number(row, key) for row in rows], dtype=float)


def one_second_count_rate(t, counts):
    """Centered one-second encoder slope; avoids the 0/100/200 TPS sampling alias."""
    before = np.maximum(t - .5, 0)
    after = np.minimum(t + .5, t[-1])
    return (np.interp(after, t, counts) - np.interp(before, t, counts)) / (after - before)


def load(run_dir):
    data = {}
    for name, file in (("Leader", "leader.csv"), ("F1", "f1.csv"), ("F2", "f2.csv")):
        with (run_dir / file).open(newline="") as stream:
            all_rows = list(csv.DictReader(stream))
        prefix, event = FIELDS[name]
        rows = [row for row in all_rows if row["event"] == event]
        assert rows, (run_dir, file)
        # The follower origin is 2–4 s before the Leader RESET. Align the
        # first moving row to the first Leader control row; UDP start lag is
        # below the wall-clock second available in the CSV.
        t = col(rows, "elapsed_s")
        t -= t[0]
        x = col(rows, prefix + "x_mm") + START[name][0]
        y = col(rows, prefix + "y_mm") + START[name][1]
        h = col(rows, prefix + "heading_deg")
        left = "status_" if name == "Leader" else "follower_"
        data[name] = dict(rows=rows, all_rows=all_rows, t=t, x=x, y=y, h=h,
                          lt=col(rows, left + "left_target_tps"),
                          rt=col(rows, left + "right_target_tps"),
                          lm=col(rows, left + "left_tps"),
                          rm=col(rows, left + "right_tps"),
                          lc=col(rows, left + "left_count"),
                          rc=col(rows, left + "right_count"),
                          lp=col(rows, left + "left_pwm"),
                          rp=col(rows, left + "right_pwm"),
                          ticks=(number(rows[0], "config_left_ticks_per_rev"),
                                 number(rows[0], "config_right_ticks_per_rev")),
                          diameter=number(rows[0], "config_wheel_diameter_mm"))
    return data


def phase_times(leader, exit_mm):
    progress = col(leader["rows"], "control_forward_mm")
    t = leader["t"]
    arc_end = 600 + 800 * math.pi / 2
    return (float(np.interp(600, progress, t)),
            float(np.interp(arc_end, progress, t)), float(t[-1]))


def follower_phase_times(follower):
    progress = col(follower["rows"], "leader_path_progress_s_mm")
    t = follower["t"]
    arc_end = 600 + 800 * math.pi / 2
    def reached(target):
        return float(np.interp(target, progress, t)) if target <= np.nanmax(progress) else float(t[-1] + 1)
    return reached(1000), reached(arc_end + 400), float(t[-1])


def shade(ax, phases):
    a, b, end = phases
    ax.axvspan(0, a, color="#dbeafe", alpha=.23, lw=0)
    ax.axvspan(a, b, color="#fef3c7", alpha=.30, lw=0)
    ax.axvspan(b, end, color="#dcfce7", alpha=.28, lw=0)
    for at in (a, b):
        ax.axvline(at, color="#64748b", alpha=.45, lw=.7)
    ax.grid(alpha=.2)


def body(ax, x, y, heading, color, alpha=.09):
    angle = math.radians(heading)
    matrix = np.array([[math.cos(angle), -math.sin(angle)],
                       [math.sin(angle), math.cos(angle)]])
    corners = BODY @ matrix.T + (x, y)
    ax.fill(corners[:, 0], corners[:, 1], color=color, alpha=alpha)
    ax.plot(corners[:, 0], corners[:, 1], "--", color=color, lw=.7)
    ax.plot(corners[:2, 0], corners[:2, 1], color=color, lw=1.5)
    ax.plot(x, y, "o", color=color, markerfacecolor="white", ms=4)


def trajectory(data, path, phases, output):
    fig, axes = plt.subplots(1, 2, figsize=(17, 7), gridspec_kw={"width_ratios": [1.25, 1]})
    ax = axes[0]
    for name, lane in (("Leader", 0), ("F1", 400), ("F2", -400)):
        s = np.linspace(-400 if lane else 0, path.end_s_mm, 500)
        p = np.array([path.pose_at_s(float(i), lane) for i in s])
        ax.plot(p[:, 0], p[:, 1], ":", color=COLORS[name], lw=1.5,
                label=f"{name} designed lane")
        d = data[name]
        ax.plot(d["x"], d["y"], color=COLORS[name], lw=2, label=f"{name} odometry")
        for idx in (0, len(d["t"]) // 2, -1):
            body(ax, d["x"][idx], d["y"][idx], d["h"][idx], COLORS[name],
                 alpha=.16 if idx == -1 else .07)
        if name != "Leader":
            tx = col(d["rows"], "control_target_x_mm")
            ty = col(d["rows"], "control_target_y_mm")
            ax.plot(tx, ty, "--", color=COLORS[name], alpha=.6, lw=1,
                    label=f"{name} instantaneous target")
    ax.set(xlabel="x (mm; on-board coordinate frame)", ylabel="y (mm)",
           title="Designed lanes / logged targets / on-board MPU-point estimates")
    ax.set_aspect("equal")
    ax.grid(alpha=.2)
    ax.legend(fontsize=8, ncol=2, loc="upper left")
    for name in ("Leader", "F1", "F2"):
        d = data[name]
        ax = axes[1]
        ax.plot(d["t"], d["h"], color=COLORS[name], lw=1.7, label=f"{name} on-board")
        if name != "Leader":
            ax.plot(d["t"], col(d["rows"], "control_target_heading_deg"),
                    "--", color=COLORS[name], alpha=.65, lw=1,
                    label=f"{name} target")
    shade(axes[1], phases)
    axes[1].set(xlabel="seconds from first moving CSV row (approx shared clock); bands=Leader phase",
                ylabel="heading (degrees)", title="Heading estimates and follower targets")
    axes[1].legend(fontsize=8, ncol=2)
    fig.suptitle(output.parent.parent.name + " — path review", fontsize=15)
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)


def formation(data, phases, output):
    fig, axes = plt.subplots(2, 2, figsize=(16, 10), sharex=True)
    lead = data["Leader"]
    axes[0, 0].plot(lead["t"], col(lead["rows"], "control_forward_mm"),
                    color=COLORS["Leader"], lw=2, label="Leader path progress")
    axes[0, 0].plot(lead["t"], col(lead["rows"], "encoder_forward_distance_mm"),
                    color="#94a3b8", lw=1, label="Leader wheel distance")
    axes[0, 0].set(ylabel="mm", title="Leader progress")
    axes[0, 0].legend()
    for name in ("F1", "F2"):
        d = data[name]
        color = COLORS[name]
        axes[0, 1].plot(d["t"], col(d["rows"], "control_leader_distance_mm"),
                        color=color, label=f"{name} MPU distance (on-board)")
        axes[1, 0].plot(d["t"], col(d["rows"], "control_longitudinal_error_mm"),
                        color=color, label=f"{name} along-lane")
        axes[1, 0].plot(d["t"], col(d["rows"], "control_lateral_error_mm"),
                        "--", color=color, label=f"{name} lateral")
        axes[1, 1].plot(d["t"], col(d["rows"], "control_heading_error_deg"),
                        color=color, label=f"{name} heading error")
    axes[0, 1].set(ylabel="mm", title="Estimated Leader–follower MPU spacing")
    axes[1, 0].set(ylabel="mm", title="Follower target tracking error")
    axes[1, 1].set(ylabel="degrees", title="Follower target heading error")
    for ax in axes.flat:
        shade(ax, phases)
        ax.legend(fontsize=8)
    for ax in axes[1]:
        ax.set_xlabel("seconds; bands show Leader straight / arc / exit")
    fig.suptitle(output.parent.parent.name + " — formation telemetry", fontsize=15)
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)


def wheels(name, d, phases, output):
    fig, axes = plt.subplots(3, 2, figsize=(17, 11), sharex=True)
    for j, side in enumerate(("left", "right")):
        target, measured, pwm, counts = (d[k] for k in (("lt", "lm", "lp", "lc") if j == 0 else ("rt", "rm", "rp", "rc")))
        rate = one_second_count_rate(d["t"], counts)
        base, slope = FEEDFORWARD[name][j]
        nominal = base + slope * np.maximum(target, 0)
        axes[0, j].plot(d["t"], target, color="#334155", lw=1.1, label="Nano target TPS")
        axes[0, j].plot(d["t"], measured, color=COLORS[name], lw=.45, alpha=.38,
                        label="raw Nano TPS (sampled)")
        axes[0, j].plot(d["t"], rate, color=COLORS[name], lw=1.5,
                        label="1 s encoder count slope")
        axes[0, j].set(ylabel="ticks/s", title=f"{name} {side} wheel: measured vs target")
        axes[1, j].plot(d["t"], pwm, color=COLORS[name], lw=1, label="logged output PWM")
        axes[1, j].plot(d["t"], nominal, "--", color="#64748b", lw=1,
                        label="nominal forward feedforward")
        axes[1, j].set(ylabel="PWM (0–255)", title="Motor output and nominal feedforward")
        axes[2, j].plot(d["t"], rate - target, color=COLORS[name], lw=1,
                        label="1 s count slope − target TPS")
        axes[2, j].axhline(0, color="#94a3b8", lw=.7)
        axes[2, j].set(ylabel="ticks/s", title="Velocity loop tracking error",
                       xlabel="seconds; blue=straight, amber=arc, green=exit")
        for i in range(3):
            shade(axes[i, j], phases)
            axes[i, j].legend(fontsize=8, loc="upper right")
    fig.suptitle(output.parent.parent.name + f" — {name} wheel velocity loop; bands={name} target-lane phase; P/I terms not logged",
                 fontsize=15)
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)


def speeds(data, phases, output):
    fig, axes = plt.subplots(3, 1, figsize=(15, 12), sharex=True)
    for ax, name in zip(axes, ("Leader", "F1", "F2")):
        d = data[name]
        left_rate = one_second_count_rate(d["t"], d["lc"])
        right_rate = one_second_count_rate(d["t"], d["rc"])
        left_mm = left_rate * math.pi * d["diameter"] / d["ticks"][0]
        right_mm = right_rate * math.pi * d["diameter"] / d["ticks"][1]
        if name == "Leader":
            requested = col(d["rows"], "plan_cruise_mm_per_second")
            label = "requested cruise"
        else:
            requested = col(d["rows"], "control_target_speed_mm_per_second")
            label = "follower target speed"
        ax.plot(d["t"], requested, ":", color="#334155", lw=1.4, label=label)
        ax.plot(d["t"], col(d["rows"], "control_left_mm_per_second"),
                "--", color="#1d4ed8", alpha=.7, lw=1, label="Pi left wheel command")
        ax.plot(d["t"], col(d["rows"], "control_right_mm_per_second"),
                "--", color="#ea580c", alpha=.7, lw=1, label="Pi right wheel command")
        ax.plot(d["t"], left_mm, color="#1d4ed8", lw=1.5,
                label="left wheel, 1 s encoder slope")
        ax.plot(d["t"], right_mm, color="#ea580c", lw=1.5,
                label="right wheel, 1 s encoder slope")
        shade(ax, phases[name])
        ax.set(ylabel="mm/s (encoder-derived)", title=name)
        ax.legend(fontsize=8, ncol=3)
    axes[-1].set_xlabel("seconds; bands show each car's target-lane straight / arc / exit")
    fig.suptitle(output.parent.parent.name + " — requested, Pi wheel commands, encoder-derived speeds",
                 fontsize=15)
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)


def video_frames(movie, duration, output):
    cap = cv2.VideoCapture(str(movie))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {movie}")
    fps = cap.get(cv2.CAP_PROP_FPS)
    total = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    actual_duration = total / fps
    # Six stages plus the last frame. Video-time to CSV-time is approximate;
    # filenames and captions retain the original video times.
    samples = [0, 10, 20, 30, 40, 50, 60, min(duration, actual_duration - 0.2)]
    frames = []
    for t in samples:
        cap.set(cv2.CAP_PROP_POS_MSEC, t * 1000)
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"Cannot read {movie} at {t:.1f}s")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (960, 540), interpolation=cv2.INTER_AREA)
        frames.append(frame)
    cap.release()
    fig, axes = plt.subplots(4, 2, figsize=(15, 18))
    for ax, t, frame in zip(axes.flat, samples, frames):
        ax.imshow(frame)
        ax.set_title(f"video +{t:.1f} s (CSV phase nearby; sync uncertainty ~1–2 s)", fontsize=11)
        ax.axis("off")
    fig.suptitle(f"{movie.name}: external, uncalibrated oblique views", fontsize=16, y=.995)
    fig.tight_layout(rect=[0, 0, 1, .975])
    fig.savefig(output, dpi=125)
    plt.close(fig)
    return {"fps": fps, "frames": int(total), "duration_s": actual_duration,
            "sample_video_times_s": samples}


def paired_video_csv(movie, data, path, output):
    cap = cv2.VideoCapture(str(movie))
    fps = cap.get(cv2.CAP_PROP_FPS)
    duration = cap.get(cv2.CAP_PROP_FRAME_COUNT) / fps
    times = [0, 20, 40, 60, min(duration - .2, data["Leader"]["t"][-1])]
    fig, axes = plt.subplots(len(times), 2, figsize=(16, 23),
                             gridspec_kw={"width_ratios": [1.15, 1]})
    for row, video_t in enumerate(times):
        cap.set(cv2.CAP_PROP_POS_MSEC, 1000 * video_t)
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"Cannot read {movie} at {video_t:.1f}s")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        axes[row, 0].imshow(frame)
        axes[row, 0].axis("off")
        axes[row, 0].set_title(f"external video +{video_t:.1f} s (no metric calibration)")
        ax = axes[row, 1]
        for name, lane in (("Leader", 0), ("F1", 400), ("F2", -400)):
            s = np.linspace(-400 if lane else 0, path.end_s_mm, 250)
            p = np.array([path.pose_at_s(float(i), lane) for i in s])
            ax.plot(p[:, 0], p[:, 1], ":", color=COLORS[name], lw=.8)
            d = data[name]
            idx = int(np.argmin(abs(d["t"] - video_t)))
            ax.plot(d["x"][:idx + 1], d["y"][:idx + 1], color=COLORS[name], lw=1.6)
            body(ax, d["x"][idx], d["y"][idx], d["h"][idx], COLORS[name], .13)
            ax.text(d["x"][idx] + 50, d["y"][idx] + 65, name, color=COLORS[name], fontsize=8)
            if name != "Leader":
                tx = number(d["rows"][idx], "control_target_x_mm")
                ty = number(d["rows"][idx], "control_target_y_mm")
                ax.plot(tx, ty, "x", color=COLORS[name], ms=6)
        ax.set_aspect("equal")
        ax.set_xlim(-600, 1950)
        ax.set_ylim(-600, 1450)
        ax.grid(alpha=.18)
        ax.set_title(f"CSV +~{video_t:.1f} s: dot=MPU estimate; cross=target")
        ax.set_xlabel("on-board x (mm)")
        ax.set_ylabel("on-board y (mm)")
    cap.release()
    fig.suptitle(movie.name + " — video and on-board path stages; time alignment approximate (±1–2 s)",
                 fontsize=14, y=.998)
    fig.tight_layout(rect=[0, 0, 1, .975])
    fig.savefig(output, dpi=125)
    plt.close(fig)


def metrics(data, phases, car_phases, video):
    result = {"phase_boundaries_leader_s": {"straight_end": phases[0],
                                             "arc_end": phases[1], "leader_stop": phases[2]},
              "phase_boundaries_by_car_s": {}, "video": video, "cars": {}}
    for name, d in data.items():
        result["phase_boundaries_by_car_s"][name] = {
            "straight_end": round(car_phases[name][0], 2),
            "arc_end": round(car_phases[name][1], 2) if car_phases[name][1] <= d["t"][-1] else None,
            "run_end": round(car_phases[name][2], 2),
        }
        car = {"onboard_endpoint_mm_deg": [round(d["x"][-1], 1), round(d["y"][-1], 1),
                                         round(d["h"][-1], 1)], "phases": {}}
        safety_prefix = "status_" if name == "Leader" else "follower_"
        faults = col(d["rows"], safety_prefix + "fault_code")
        car["health"] = {
            "nonzero_fault_rows": int(np.sum(faults != 0)),
            "preflight_not_one_rows": int(np.sum(col(d["rows"], safety_prefix + "encoder_preflight") != 1)),
            "imu_missing_rows": int(np.sum(col(d["rows"], safety_prefix + "imu_present") != 1)),
            "imu_uncalibrated_rows": int(np.sum(col(d["rows"], safety_prefix + "imu_calibrated") != 1)),
        }
        if name != "Leader":
            car["health"]["tracking_unsafe_rows"] = sum(
                row["control_tracking_safe"].lower() != "true" for row in d["rows"])
            car["health"]["nonempty_stop_reason_rows"] = sum(
                bool(row["control_stop_reason"]) for row in d["rows"])
            car["health"]["max_leader_frame_age_s"] = round(float(np.nanmax(
                col(d["rows"], "leader_frame_age_s"))), 3)
        for side, tk, mk, pk, ticks in (("left", "lt", "lm", "lp", d["ticks"][0]),
                                        ("right", "rt", "rm", "rp", d["ticks"][1])):
            target, measured, pwm = d[tk], d[mk], d[pk]
            counts = d["lc"] if side == "left" else d["rc"]
            rate = one_second_count_rate(d["t"], counts)
            # Exclude near-zero target and the first two seconds of startup
            # so ratio is not dominated by acceleration or division by zero.
            active = (target >= 50) & (d["t"] >= 3) & (d["t"] <= d["t"][-1] - 1)
            car[side] = {
                "median_target_tps_active": round(float(np.median(target[active])), 1),
                "median_count_slope_tps_active": round(float(np.median(rate[active])), 1),
                "median_count_slope_over_target_active": round(float(np.median(rate[active] / target[active])), 3),
                "p95_abs_tracking_error_tps_active": round(float(np.percentile(abs(rate[active] - target[active]), 95)), 1),
                "max_logged_pwm": int(np.nanmax(pwm)),
                "mm_per_tick": math.pi * d["diameter"] / ticks,
            }
        start, bend, end = car_phases[name]
        for phase, low, high in (("straight", 0, start), ("arc", start, bend),
                                 ("exit", bend, end)):
            section = {"duration_s": round(max(0, min(high, end) - low), 2)}
            for side, tk, ck, pk in (("left", "lt", "lc", "lp"),
                                     ("right", "rt", "rc", "rp")):
                target = d[tk]
                rate = one_second_count_rate(d["t"], d[ck])
                mask = (d["t"] >= max(low, 2)) & (d["t"] < min(high, end - .5)) & (target >= 50)
                section[side] = ({"median_rate_over_target": round(float(np.median(rate[mask] / target[mask])), 3),
                                  "median_pwm": int(np.median(d[pk][mask])),
                                  "max_pwm": int(np.max(d[pk][mask]))} if mask.any() else None)
            car["phases"][phase] = section
        if name != "Leader":
            for field, key in (("lateral_error", "control_lateral_error_mm"),
                               ("longitudinal_error", "control_longitudinal_error_mm"),
                               ("heading_error", "control_heading_error_deg")):
                values = col(d["rows"], key)
                car[field] = {"median_abs": round(float(np.nanmedian(abs(values))), 2),
                              "max_abs": round(float(np.nanmax(abs(values))), 2)}
            car["min_estimated_leader_mpu_distance_mm"] = round(float(np.nanmin(
                col(d["rows"], "control_leader_distance_mm"))), 1)
        result["cars"][name] = car
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--movies", type=Path, default=Path.home() / "Downloads")
    args = parser.parse_args()
    for run, (movie_name, exit_mm, speed) in RUNS.items():
        run_dir = HERE / run
        output = run_dir / "review"
        output.mkdir(exist_ok=True)
        data = load(run_dir)
        path = TurnPath(TurnPathReference(600, 800, exit_mm, 0, 0, 0))
        phases = phase_times(data["Leader"], exit_mm)
        car_phases = {"Leader": phases,
                      "F1": follower_phase_times(data["F1"]),
                      "F2": follower_phase_times(data["F2"])}
        trajectory(data, path, phases, output / "trajectory_targets.png")
        formation(data, phases, output / "formation_telemetry.png")
        speeds(data, car_phases, output / "speed_commands.png")
        for name in data:
            wheels(name, data[name], car_phases[name], output / f"{name.lower()}_wheel_loop.png")
        movie = args.movies / movie_name
        video = video_frames(movie, 72 if exit_mm == 200 else 65, output / "video_stages.jpg")
        paired_video_csv(movie, data, path, output / "video_csv_pairs.jpg")
        report = metrics(data, phases, car_phases, video)
        report["route"] = {"straight_mm": 600, "radius_mm": 800,
                           "exit_mm": exit_mm, "leader_command_mm_s": speed,
                           "follower_lag_mm": 400, "lane_offsets_mm": [400, -400]}
        report["video_file"] = str(movie)
        (output / "metrics.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        print(run, output)


if __name__ == "__main__":
    main()
