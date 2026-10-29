"""Place video frames beside the time-matched CSV estimates.

Usage (from project root):
  MPLBACKEND=Agg .venv/bin/python analysis/2026-10-29-f1-f2-theory/sync_video_csv.py \
      /path/to/IMG_8103.MOV --run-dir analysis/2026-10-29-f1-f2-theory/real_run_084014Z \
      --video-start-utc 2026-10-29T08:36:51Z --route-start-video-s 203 \
      --times 203 208 213 218 223 230 --crop 80 1900 2080 3400

CSV wall times are rounded to seconds. Matching assumes the phone and Pi clocks
are aligned; their offset has not been measured. Video pixels are not ground mm.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta, timezone
from pathlib import Path
import sys

import cv2
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pi_rover"))
from wave_path import WavePath, WavePathReference  # noqa: E402
from vehicle_footprint import draw_body  # noqa: E402

DEFAULT_RUN = Path(__file__).resolve().parent / "real_run_074722Z"
DEFAULT_START = datetime(2026, 10, 29, 7, 47, 34, tzinfo=timezone.utc)
COLORS = {"leader": "#252a30", "f1": "#2478b3", "f2": "#c66b21"}


def load(run: Path, name: str, event: str) -> list[dict[str, str]]:
    with (run / f"{name}.csv").open(newline="") as stream:
        return [r for r in csv.DictReader(stream) if r["event"] == event]


def at_time(rows: list[dict[str, str]], when: datetime) -> tuple[int, dict[str, str]]:
    second = when.strftime("%Y-%m-%dT%H:%M:%SZ")
    matches = [i for i, r in enumerate(rows) if r["wall_time_utc"] == second]
    if not matches:
        raise ValueError(f"No CSV sample at {second}")
    index = matches[len(matches) // 2]
    return index, rows[index]


def coordinate(rows: list[dict[str, str]], name: str, axis: str) -> list[float]:
    if name == "leader":
        return [float(r[f"status_{axis}_mm"]) for r in rows]
    offset = -400 if axis == "x" else (400 if name == "f1" else -400)
    return [float(r[f"follower_{axis}_mm"]) + offset for r in rows]


def frame_at(capture: cv2.VideoCapture, seconds: int,
             crop: tuple[int, int, int, int]):
    capture.set(cv2.CAP_PROP_POS_MSEC, seconds * 1000)
    ok, frame = capture.read()
    if not ok:
        raise RuntimeError(f"Cannot read video at {seconds} s")
    # Crop to the three cars. This is camera imagery, not a calibrated map.
    left, top, right, bottom = crop
    return cv2.cvtColor(frame[top:bottom, left:right], cv2.COLOR_BGR2RGB)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--video-start-utc", default=DEFAULT_START.isoformat())
    parser.add_argument("--times", nargs="+", type=int, default=[0, 4, 8, 12])
    parser.add_argument("--route-start-video-s", type=int, default=0)
    parser.add_argument("--crop", nargs=4, type=int, default=[100, 1050, 2100, 2400],
                        metavar=("LEFT", "TOP", "RIGHT", "BOTTOM"))
    args = parser.parse_args()
    start = datetime.fromisoformat(args.video_start_utc.replace("Z", "+00:00"))
    if start.tzinfo is None:
        raise ValueError("--video-start-utc must include a timezone")
    start = start.astimezone(timezone.utc)
    leader = load(args.run_dir, "leader", "control")
    f1 = load(args.run_dir, "f1", "control_moving")
    f2 = load(args.run_dir, "f2", "control_moving")
    data = {"leader": leader, "f1": f1, "f2": f2}
    route = WavePath(WavePathReference(1500, 0, 50, 3, 0, 0, 0))
    plan = [route.pose_at_s(route.end_s_mm * i / 500) for i in range(501)]

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {args.video}")
    fig, axes = plt.subplots(len(args.times), 2, figsize=(17, 4.25 * len(args.times)),
                             gridspec_kw={"width_ratios": [1.0, 1.1]})
    for row_number, video_second in enumerate(args.times):
        when = start + timedelta(seconds=video_second)
        selections = {name: at_time(rows, when) for name, rows in data.items()}
        ax_video, ax_map = axes[row_number]
        ax_video.imshow(frame_at(capture, video_second, tuple(args.crop)))
        ax_video.axis("off")
        ax_video.set_title(
            f"Video +{video_second} s / route +{video_second - args.route_start_video_s} s | "
            f"{when:%H:%M:%S} UTC | left F1, middle Leader, right F2", fontsize=10,
        )

        ax_map.plot([p[0] for p in plan], [p[1] for p in plan],
                    ls="--", lw=1.2, color=COLORS["leader"], alpha=0.7)
        for name in ("leader", "f1", "f2"):
            index, sample = selections[name]
            subset = data[name][:index + 1]
            x = coordinate(subset, name, "x")
            y = coordinate(subset, name, "y")
            color = COLORS[name]
            ax_map.plot(x, y, color=color, lw=2.0)
            heading_field = "status_heading_deg" if name == "leader" else "follower_heading_deg"
            draw_body(ax_map, x[-1], y[-1], float(sample[heading_field]),
                      color, prominent=True)
            if name != "leader":
                tx = [float(r["control_target_x_mm"]) for r in subset]
                ty = [float(r["control_target_y_mm"]) for r in subset]
                ax_map.plot(tx, ty, color=color, ls="--", lw=1.2)
                ax_map.scatter(tx[-1], ty[-1], color=color, marker="x", s=65, zorder=5)
        l = selections["leader"][1]
        a = selections["f1"][1]
        b = selections["f2"][1]
        ax_map.set(xlim=(-580, 1730), ylim=(-560, 690),
                   title=(f"Leader x={float(l['status_x_mm']):.0f} mm  |  "
                          f"F1 target gap={float(a['control_rho_mm']):.0f} mm  |  "
                          f"F2 target gap={float(b['control_rho_mm']):.0f} mm"),
                   xlabel="x (mm)" if row_number == len(args.times) - 1 else "",
                   ylabel="y (mm)")
        ax_map.set_aspect("equal", adjustable="box")
        ax_map.grid(alpha=0.16)
    capture.release()
    fig.suptitle("Same moments: camera image vs planned route, controller targets, and onboard odometry", fontsize=15)
    note = (" At the final frame, F1 and Leader appear nearly touching, while CSV predicts ~203 mm body gap."
            if args.run_dir.name == "real_run_084014Z" else "")
    fig.text(0.5, 0.018,
             "Map: solid = onboard odometry at MPU anchor; dot = MPU coordinate; dashed body = 250 × 150 mm (+100 nose / −150 tail).\n"
             "Dashed paths = plan/targets; x = current target. Video is uncalibrated; time match assumes phone/Pi clocks agree." + note,
             ha="center", va="center", fontsize=8.5)
    fig.tight_layout(rect=(0, 0.05, 1, 0.97), h_pad=2.2)
    output = args.run_dir / "video_csv_sync.png"
    fig.savefig(output, dpi=170)
    print(output)


if __name__ == "__main__":
    main()
