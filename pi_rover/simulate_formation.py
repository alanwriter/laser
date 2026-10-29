#!/usr/bin/env python3
"""Offline ideal-kinematics comparison of A/B and normal-follow controllers.

No serial port or UDP socket is opened.  The same leader packet builder and
follower packet parser used in the live controllers are exercised each step.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, replace
from pathlib import Path

from follower_formation import (
    FormationPlan, FormationTracker, PathFormationTracker, parse_leader_frame,
)
from leader_formation import (
    RoverStatus, WavePlan, leader_state_payload, wave_control,
)
from wave_path import WavePath, WavePathReference


def status(x: float = 0.0, y: float = 0.0, heading: float = 0.0) -> RoverStatus:
    return RoverStatus(
        mode="velocity", x_mm=x, y_mm=y, heading_deg=heading,
        left_count=0, right_count=0, left_tps=0.0, right_tps=0.0,
        left_target_tps=0.0, right_target_tps=0.0,
        left_pwm=0, right_pwm=0, fault_code=0, encoder_preflight=1,
        imu_present=1, imu_calibrated=1, path_number=0, path_step=0,
    )


def step_rover(rover: RoverStatus, left: int, right: int, dt: float, track: float) -> RoverStatus:
    """Integrate an ideal differential-drive rover over one held setpoint."""
    speed = (left + right) / 2.0
    yaw_rate = (right - left) / track
    old_heading = math.radians(rover.heading_deg)
    new_heading = old_heading + yaw_rate * dt
    if abs(yaw_rate) < 1e-12:
        dx = speed * math.cos(old_heading) * dt
        dy = speed * math.sin(old_heading) * dt
    else:
        dx = speed * (math.sin(new_heading) - math.sin(old_heading)) / yaw_rate
        dy = speed * (math.cos(old_heading) - math.cos(new_heading)) / yaw_rate
    return replace(
        rover, x_mm=rover.x_mm + dx, y_mm=rover.y_mm + dy,
        heading_deg=math.degrees(new_heading),
    )


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return round(ordered[round(fraction * (len(ordered) - 1))], 2)


def summarize(rows: list[dict[str, object]], prefix: str) -> dict[str, object]:
    active = [r for r in rows if r.get(f"{prefix}_rho_mm") is not None]
    errors = [float(r[f"{prefix}_rho_mm"]) for r in active]
    lateral = [abs(float(r[f"{prefix}_lateral_error_mm"])) for r in active]
    progress = [abs(float(r[f"{prefix}_progress_error_mm"])) for r in active]
    path_cross_track = [abs(float(r[f"{prefix}_path_cross_track_mm"])) for r in active]
    yaw_rates = [abs((float(r[f"{prefix}_right_mm_s"]) - float(r[f"{prefix}_left_mm_s"])) / 130.0) for r in active]
    return {
        "active_samples": len(active),
        "median_target_error_mm": percentile(errors, 0.5),
        "p90_target_error_mm": percentile(errors, 0.9),
        "median_abs_lateral_error_mm": percentile(lateral, 0.5),
        "median_abs_path_cross_track_mm": percentile(path_cross_track, 0.5),
        "median_abs_longitudinal_or_progress_error_mm": percentile(progress, 0.5),
        "final_target_error_mm": round(errors[-1], 2) if errors else None,
        "fraction_abs_commanded_yaw_at_least_0_23_rad_s": round(sum(y >= 0.23 for y in yaw_rates) / len(yaw_rates), 3) if yaw_rates else None,
        "stopped_by_safety": next((
            {"time_s": r["time_s"], "reason": r[f"{prefix}_stop_reason"]}
            for r in rows if r.get(f"{prefix}_stop_reason")
        ), None),
    }


def simulate(plan: WavePlan, normal_follow: bool = False) -> tuple[list[dict[str, object]], dict[str, object], list[dict[str, object]]]:
    dt = plan.control_period_s
    leader = status()
    origin = leader
    f1 = status()
    f2 = status()
    f1_tracker = (PathFormationTracker(FormationPlan()) if normal_follow
                  else FormationTracker(FormationPlan()))
    f2_tracker = PathFormationTracker(FormationPlan(
        offset_x_mm=-400.0, offset_y_mm=-400.0,
        follower_start_x_mm=-400.0, follower_start_y_mm=-400.0,
    ))
    base = WavePathReference(
        plan.length_mm, plan.lead_in_mm, plan.amplitude_mm, plan.cycles,
        0.0, 0.0, 0.0,
    )
    geometry = WavePath(base)
    rows: list[dict[str, object]] = []
    samples: list[dict[str, object]] = []
    f1_running = f2_running = True
    f1_stop = f2_stop = ""
    leader_result = "deadline"
    for tick in range(int(plan.max_runtime_s / dt) + 1):
        t = tick * dt
        lead_control = wave_control(leader, origin, plan)
        progress_x = min(max(lead_control.forward_mm, 0.0), plan.length_mm)
        path = replace(base, progress_s_mm=geometry.s_at_x(progress_x))
        lead_speed = (lead_control.left_mm_per_second + lead_control.right_mm_per_second) / 2.0
        lead_yaw = (lead_control.right_mm_per_second - lead_control.left_mm_per_second) / plan.wheel_track_mm
        packet = leader_state_payload(leader, lead_speed, lead_yaw, path=path)
        encoded = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        parsed = parse_leader_frame(encoded, "192.168.1.166", t)
        f1_control = f2_control = None
        if tick >= 2 and f1_running:
            f1_control = f1_tracker.control(parsed, f1)
            if not f1_control.tracking_safe:
                f1_running = False
                f1_stop = f1_control.stop_reason
        if tick >= 2 and f2_running:
            f2_control = f2_tracker.control(parsed, f2)
            if not f2_control.tracking_safe:
                f2_running = False
                f2_stop = f2_control.stop_reason
        row: dict[str, object] = {
            "time_s": round(t, 2),
            "leader_x_mm": round(leader.x_mm, 2),
            "leader_y_mm": round(leader.y_mm, 2),
            "leader_heading_deg": round(leader.heading_deg, 2),
            "leader_target_y_mm": round(lead_control.desired_lateral_mm, 2),
            "leader_left_mm_s": lead_control.left_mm_per_second,
            "leader_right_mm_s": lead_control.right_mm_per_second,
            "path_progress_s_mm": round(path.progress_s_mm, 2),
            "f1_x_mm": round(f1.x_mm - 400, 2),
            "f1_y_mm": round(f1.y_mm + 400, 2),
            "f1_heading_deg": round(f1.heading_deg, 2),
            "f2_x_mm": round(f2.x_mm - 400, 2),
            "f2_y_mm": round(f2.y_mm - 400, 2),
            "f2_heading_deg": round(f2.heading_deg, 2),
            "f1_stop_reason": f1_stop,
            "f2_stop_reason": f2_stop,
        }
        row["f1_path_cross_track_mm"] = round(geometry.project(f1.x_mm - 400, f1.y_mm + 400, 400)[1], 2)
        row["f2_path_cross_track_mm"] = round(geometry.project(f2.x_mm - 400, f2.y_mm - 400, -400)[1], 2)
        for name, control in (("f1", f1_control), ("f2", f2_control)):
            row.update({
                f"{name}_target_x_mm": round(control.target_x_mm, 2) if control else None,
                f"{name}_target_y_mm": round(control.target_y_mm, 2) if control else None,
                f"{name}_target_heading_deg": round(control.target_heading_deg, 2) if control else None,
                f"{name}_rho_mm": round(control.rho_mm, 2) if control else None,
                f"{name}_lateral_error_mm": round(control.lateral_error_mm, 2) if control else None,
                f"{name}_progress_error_mm": round(control.longitudinal_error_mm, 2) if control else None,
                f"{name}_heading_error_deg": round(control.heading_error_deg, 2) if control else None,
                f"{name}_left_mm_s": control.left_mm_per_second if control else 0,
                f"{name}_right_mm_s": control.right_mm_per_second if control else 0,
            })
        rows.append(row)
        if tick in (2, 100, 250, 500):
            samples.append({
                "time_s": round(t, 1), "leader_packet": packet,
                "f1_control": asdict(f1_control) if f1_control else None,
                "f2_control": asdict(f2_control) if f2_control else None,
                "packet_bytes": len(encoded),
            })
        if (
            lead_control.forward_mm >= plan.length_mm
            and abs(lead_control.lateral_mm) <= plan.completion_lateral_tolerance_mm
            and abs(lead_control.heading_error_deg) <= plan.completion_heading_tolerance_deg
        ):
            leader_result = "completed"
            break
        if lead_control.forward_mm > plan.length_mm + plan.max_finish_overrun_mm:
            leader_result = "finish_overrun"
            break
        leader = step_rover(
            leader, lead_control.left_mm_per_second,
            lead_control.right_mm_per_second, dt, plan.wheel_track_mm,
        )
        if f1_control and f1_running:
            f1 = step_rover(
                f1, f1_control.left_mm_per_second,
                f1_control.right_mm_per_second, dt, 130.0,
            )
        if f2_control and f2_running:
            f2 = step_rover(
                f2, f2_control.left_mm_per_second,
                f2_control.right_mm_per_second, dt, 130.0,
            )
    summary = {
        "model": "Ideal differential-drive integration of actual Python wheel commands at 0.1 s; no slip, delay, IMU drift or motor dynamics",
        "leader_plan": asdict(plan),
        "leader_result": leader_result,
        "duration_s": rows[-1]["time_s"],
        "leader_final_x_mm": rows[-1]["leader_x_mm"],
        "leader_final_y_mm": rows[-1]["leader_y_mm"],
        ("f1_path_progress" if normal_follow else "f1_old_body_offset"): summarize(rows, "f1"),
        "f2_new_path_progress": summarize(rows, "f2"),
        "controller_mode": "normal-follow-v0.1.0" if normal_follow else "historical-ab",
    }
    return rows, summary, samples


def plot(rows: list[dict[str, object]], plan: WavePlan, destination: Path,
         normal_follow: bool = False) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(3, 1, figsize=(11, 11), constrained_layout=True)
    ax = axes[0]
    ax.plot([r["leader_x_mm"] for r in rows], [r["leader_y_mm"] for r in rows], label="Leader simulated", color="black")
    ax.plot([r["leader_x_mm"] for r in rows], [r["leader_target_y_mm"] for r in rows], label="Leader planned", color="gray", linestyle="--")
    f1_label = "F1 path" if normal_follow else "F1 old"
    for name, color, label in (("f1", "#d95f02", f1_label), ("f2", "#1b9e77", "F2 path")):
        ax.plot([r[f"{name}_x_mm"] for r in rows], [r[f"{name}_y_mm"] for r in rows], label=label, color=color)
        active = [r for r in rows if r[f"{name}_target_x_mm"] is not None]
        ax.plot([r[f"{name}_target_x_mm"] for r in active], [r[f"{name}_target_y_mm"] for r in active], color=color, alpha=.5, linestyle=":", label=label + " target")
    ax.set(xlabel="x (mm)", ylabel="y (mm)", title=f"1 leader + {f1_label} + F2 path | {plan.length_mm:.0f} mm, {plan.cycles} cycles, amplitude {plan.amplitude_mm:.0f} mm")
    ax.axis("equal")
    ax.grid(True, alpha=.25)
    ax.legend(ncol=3, fontsize=9)
    ax = axes[1]
    for name, color, label in (("f1", "#d95f02", "F1 left translated route"), ("f2", "#1b9e77", "F2 right translated route")):
        active = [r for r in rows if r[f"{name}_rho_mm"] is not None]
        ax.plot([r["time_s"] for r in active], [abs(float(r[f"{name}_path_cross_track_mm"])) for r in active], color=color, label=label)
    ax.set(xlabel="time (s)", ylabel="absolute cross-track error (mm)", title="Same path metric, mirrored lanes")
    ax.grid(True, alpha=.25)
    ax.legend()
    ax = axes[2]
    for name, color, label in (("f1", "#d95f02", f"{f1_label} target error"), ("f2", "#1b9e77", "F2 path target error")):
        active = [r for r in rows if r[f"{name}_rho_mm"] is not None]
        ax.plot([r["time_s"] for r in active], [r[f"{name}_rho_mm"] for r in active], color=color, label=label)
    ax.set(xlabel="time (s)", ylabel="distance to each controller's target (mm)", title="Target errors")
    ax.grid(True, alpha=.25)
    ax.legend()
    figure.savefig(destination, dpi=160)
    plt.close(figure)


def plot_control_terms(rows: list[dict[str, object]], plan: WavePlan, destination: Path) -> None:
    """Show why F2 can hold the route yet fall behind at tight turns."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = WavePath(WavePathReference(
        plan.length_mm, plan.lead_in_mm, plan.amplitude_mm,
        plan.cycles, 0.0, 0.0, 0.0,
    ))
    active = [r for r in rows if r["f2_rho_mm"] is not None]
    times = [float(r["time_s"]) for r in active]
    progress = [float(r["f2_progress_error_mm"]) for r in active]
    lateral = [float(r["f2_lateral_error_mm"]) for r in active]
    leader_speed = [(float(r["leader_left_mm_s"]) + float(r["leader_right_mm_s"])) / 2 for r in active]
    f2_speed = [(float(r["f2_left_mm_s"]) + float(r["f2_right_mm_s"])) / 2 for r in active]
    f2_yaw = [abs(float(r["f2_right_mm_s"]) - float(r["f2_left_mm_s"])) / 130.0 for r in active]
    curvature_speed_limit = []
    for r in active:
        actual_s, _ = path.project(float(r["f2_x_mm"]), float(r["f2_y_mm"]), -400.0)
        _, _, _, curvature = path.pose_at_s(actual_s, -400.0)
        curvature_speed_limit.append(min(
            75.0,
            0.25 / abs(curvature) if abs(curvature) > 1e-9 else 75.0,
            75.0 / (1.0 + 130.0 * abs(curvature) / 2.0),
        ))

    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    axes[0].plot(times, progress, color="#1b9e77", label="F2 along-path lag")
    axes[0].plot(times, lateral, color="#7570b3", label="F2 cross-track error")
    axes[0].axhline(0, color="gray", linewidth=.7)
    axes[0].set(ylabel="error (mm)", title="F2 errors: close to its lane, increasingly behind the desired progress")
    axes[0].legend()

    axes[1].plot(times, leader_speed, color="black", label="Leader commanded forward speed")
    axes[1].plot(times, f2_speed, color="#1b9e77", label="F2 commanded forward speed")
    axes[1].plot(times, curvature_speed_limit, color="#d95f02", linestyle="--", label="F2 path speed limit from curvature")
    axes[1].set(ylabel="speed (mm/s)", title="Tight turns force F2 to slow; it catches up between turns")
    axes[1].legend()

    axes[2].plot(times, f2_yaw, color="#1b9e77", label="F2 commanded |yaw rate|")
    axes[2].axhline(.23, color="#d95f02", linestyle="--", label="near-limit threshold: 0.23 rad/s")
    axes[2].axhline(.25, color="black", linestyle=":", label="configured limit: 0.25 rad/s")
    near_limit = sum(rate >= .23 for rate in f2_yaw)
    axes[2].set(xlabel="time (s)", ylabel="yaw rate (rad/s)", title=f"{near_limit} of {len(f2_yaw)} control samples are at or above 0.23 rad/s")
    axes[2].legend()
    for ax in axes:
        ax.grid(True, alpha=.25)
        for marker in (10, 25, 30):
            ax.axvline(marker, color="gray", alpha=.25, linewidth=.8)
    figure.savefig(destination, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--length-mm", type=float, default=1500.0)
    parser.add_argument("--amplitude-mm", type=float, default=50.0)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--speed-mm-s", type=float, default=45.0)
    parser.add_argument("--mode", choices=("ab", "normal"), default="ab")
    parser.add_argument("--out-dir", type=Path)
    args = parser.parse_args()
    plan = WavePlan(
        length_mm=args.length_mm, amplitude_mm=args.amplitude_mm,
        cycles=args.cycles, cruise_mm_per_second=args.speed_mm_s,
    )
    out_dir = args.out_dir or (Path(__file__).resolve().parents[1] / "analysis" /
                               ("normal-follow-v0.1.0-simulation" if args.mode == "normal"
                                else "2026-10-29-f1-f2-theory"))
    out_dir.mkdir(parents=True, exist_ok=True)
    normal_follow = args.mode == "normal"
    rows, summary, samples = simulate(plan, normal_follow)
    with (out_dir / "simulation.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (out_dir / "sample_packets_and_controls.json").write_text(json.dumps(samples, indent=2) + "\n", encoding="utf-8")
    plot(rows, plan, out_dir / "comparison.png", normal_follow)
    plot_control_terms(rows, plan, out_dir / "control_explained.png")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
