#!/usr/bin/env python3
"""Reconstruct an approximate ground-plane rover axis from a fixed camera.

Seven hand-marked wheel/floor-contact rows plus the 20 cm floor marks provide
an independent x axis. Lateral pixels are scaled by the measured 160 mm wheel
outer width. The point tracked is the lower body/wheel-center image proxy,
not a visible MPU marker.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
VIDEO = Path("/Users/alansmacbook/Downloads/20261117_ALAN0531.MP4")
CSV = HERE / "leader.csv"
IMAGE_SIZE = (540, 960)
VIDEO_START_S = 4.0  # onset in video; 39.4 s CSV motion ends around video 43 s
PX_PER_MM_AT_ORIGIN = 0.54  # ~86 px wheel outer width / measured 160 mm
FLOOR_TICK_ROWS = np.array([171, 203, 242, 285, 334, 387, 448, 520, 605, 706, 825], float)
FLOOR_TICK_X_MM = np.arange(len(FLOOR_TICK_ROWS)) * 200.0
WHEEL_CONTACT_VIDEO_S = np.array([4, 9, 15, 24, 30, 39, 43], float)
WHEEL_CONTACT_IMAGE_ROW = np.array([180, 230, 290, 425, 535, 760, 865], float)


def ground_projection() -> tuple[float, float, float]:
    # 1-D projective camera fit: v=(a+b*x)/(1+c*x).
    a, b, c = np.linalg.lstsq(
        np.column_stack((np.ones_like(FLOOR_TICK_X_MM), FLOOR_TICK_X_MM,
                         -FLOOR_TICK_X_MM * FLOOR_TICK_ROWS)),
        FLOOR_TICK_ROWS, rcond=None)[0]
    return float(a), float(b), float(c)


def make_background(capture: cv2.VideoCapture) -> np.ndarray:
    frames = []
    for seconds in np.arange(0, 46, 0.75):
        capture.set(cv2.CAP_PROP_POS_MSEC, float(seconds * 1000))
        okay, frame = capture.read()
        if okay:
            frames.append(cv2.resize(frame, IMAGE_SIZE))
    return np.median(np.stack(frames), axis=0).astype(np.uint8)


def floor_centerline(background: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    hsv = cv2.cvtColor(background, cv2.COLOR_BGR2HSV)
    rows, centers = [], []
    for row in np.arange(180, 950, 10):
        patch = hsv[row - 2:row + 3, 285:355]
        yellow = ((patch[:, :, 0] > 8) & (patch[:, :, 0] < 40) &
                  (patch[:, :, 1] > 60) & (patch[:, :, 2] > 55))
        _, xs = np.where(yellow)
        if len(xs) >= 8:
            rows.append(row)
            centers.append(285 + np.median(xs))
    return np.asarray(rows), np.asarray(centers)


def track_frame(frame: np.ndarray, background: np.ndarray) -> tuple[float, float, tuple[int, ...]] | None:
    image = cv2.resize(frame, IMAGE_SIZE)
    difference = cv2.cvtColor(cv2.absdiff(image, background), cv2.COLOR_BGR2GRAY)
    mask = (difference > 28).astype(np.uint8) * 255
    # Thin floor tape is removed by vertical opening; the moving chassis remains.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    candidates = [(area, index, x, y, width, height)
                  for index, (x, y, width, height, area) in enumerate(stats)
                  if index and area > 2500 and 40 < width < 320 and height > 40]
    if not candidates:
        return None
    _, label, x, y, width, height = max(candidates)
    body = labels[int(y + 0.55 * height):min(960, int(y + 0.82 * height)), x:x + width] == label
    _, xs = np.where(body)
    if len(xs) < 200:
        return None
    return float(x + np.median(xs)), float(y + 0.68 * height), (x, y, width, height)


def main() -> None:
    rows = list(csv.DictReader(CSV.open(encoding="utf-8")))
    controls = [row for row in rows if row["event"] == "control"]
    elapsed = np.array([float(row["elapsed_s"]) for row in controls])
    odom_x = np.array([float(row["corrected_x_mm"]) for row in controls])
    odom_y = np.array([float(row["corrected_y_mm"]) for row in controls])
    raw_x = np.array([float(row["raw_x_mm"]) for row in controls])
    raw_y = np.array([float(row["raw_y_mm"]) for row in controls])
    a, b, c = ground_projection()
    tick_pred = (a + b * FLOOR_TICK_X_MM) / (1 + c * FLOOR_TICK_X_MM)
    ground_x = (WHEEL_CONTACT_IMAGE_ROW - a) / (b - c * WHEEL_CONTACT_IMAGE_ROW)
    ground_x -= ground_x[0]  # axle offset cancels between initial and final ground points

    capture = cv2.VideoCapture(str(VIDEO))
    background = make_background(capture)
    line_rows, line_centers = floor_centerline(background)
    samples = []
    overlay = background.copy()
    for seconds in np.arange(4.0, 43.01, 0.5):
        capture.set(cv2.CAP_PROP_POS_MSEC, float(seconds * 1000))
        okay, frame = capture.read()
        if not okay:
            continue
        found = track_frame(frame, background)
        if found is None:
            continue
        body_x, axis_image_row, bbox = found
        elapsed_s = np.clip(seconds - VIDEO_START_S, 0, elapsed[-1])
        x_mm = float(np.interp(seconds, WHEEL_CONTACT_VIDEO_S, ground_x))
        line_x = float(np.interp(axis_image_row, line_rows, line_centers))
        scale = PX_PER_MM_AT_ORIGIN / (1 + c * x_mm)
        video_y = (body_x - line_x) / scale
        samples.append((seconds, elapsed_s, x_mm, video_y, body_x, line_x,
                        axis_image_row, scale, *bbox))
        cv2.circle(overlay, (round(body_x), round(axis_image_row)), 3, (0, 0, 255), -1)
    capture.release()

    fields = ["video_s", "csv_elapsed_s", "ground_x_mm", "video_lateral_mm",
              "body_center_px", "floor_line_px", "body_proxy_row_px", "px_per_mm",
              "bbox_x", "bbox_y", "bbox_width", "bbox_height"]
    with (HERE / "video_track.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(fields)
        writer.writerows(samples)
    cv2.imwrite(str(HERE / "video_axis_trace.jpg"), overlay)

    data = np.asarray(samples)
    vx, vy = data[:, 2], data[:, 3]
    # Reject one segmentation outlier at ~5 s (auto-exposure changed the mask).
    keep = np.ones(len(vx), dtype=bool)
    for index in range(2, len(vx) - 2):
        neighborhood = np.r_[vy[index - 2:index], vy[index + 1:index + 3]]
        if abs(vy[index] - np.median(neighborhood)) > 25:
            keep[index] = False
    vx, vy = vx[keep], vy[keep]
    theory = -70 * np.sin(3 * math.pi * vx / 2000) ** 3
    # Fit drift axis and waveform gain; retain original offsets in the top panel.
    intercept, drift_per_mm, wave_gain = np.linalg.lstsq(
        np.column_stack((np.ones_like(vx), vx, theory)), vy, rcond=None)[0]
    axis = intercept + drift_per_mm * vx
    video_centered = vy - axis
    odom_at_video = np.interp(data[keep, 1], elapsed, odom_y)
    video_error = video_centered - theory
    odom_error = odom_at_video - theory
    print("video samples", len(vx))
    print("ground tick fit max residual px", np.max(abs(tick_pred - FLOOR_TICK_ROWS)))
    print("manual ground-x vs corrected odom x (mm)")
    for seconds, ground in zip(WHEEL_CONTACT_VIDEO_S, ground_x):
        onboard = np.interp(np.clip(seconds - VIDEO_START_S, 0, elapsed[-1]), elapsed, odom_x)
        print(round(seconds, 1), round(ground), round(onboard), round(ground - onboard))
    print("fitted axis intercept/end/drift mm", round(intercept, 1),
          round(intercept + drift_per_mm * 2000, 1), round(drift_per_mm * 2000, 1))
    print("video waveform gain", round(wave_gain, 3))
    print("video raw y range/end", round(float(vy.min()), 1), round(float(vy.max()), 1),
          round(float(vy[-1]), 1))
    print("video centered y range/end", round(float(video_centered.min()), 1),
          round(float(video_centered.max()), 1), round(float(video_centered[-1]), 1))
    print("video centered-theory MAE RMSE", round(float(np.mean(abs(video_error))), 1),
          round(float(np.sqrt(np.mean(video_error ** 2))), 1))
    print("odom-theory MAE RMSE at video x", round(float(np.mean(abs(odom_error))), 1),
          round(float(np.sqrt(np.mean(odom_error ** 2))), 1))
    print("video raw-odom median/end", round(float(np.median(vy - odom_at_video)), 1),
          round(float(vy[-1] - odom_at_video[-1]), 1))

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    dense_x = np.linspace(0, 2000, 500)
    dense_theory = -70 * np.sin(3 * math.pi * dense_x / 2000) ** 3
    axes[0].plot(dense_x, dense_theory, color="#555555", linestyle="--", label="Planned MPU path")
    axes[0].plot(odom_x, odom_y, color="#c05a16", lw=1.6, label="Nano pose ×1.1483")
    axes[0].plot(raw_x, raw_y, color="#e1a56c", lw=1.2, alpha=.7, label="Raw Nano pose")
    axes[0].plot(vx, vy, color="#1865a7", lw=2, marker=".", ms=2,
                 label="Video body-axis proxy (floor scale)")
    axes[0].plot(dense_x, intercept + drift_per_mm * dense_x, color="#1865a7",
                 linestyle=":", lw=1.5, label="Video fitted drift axis")
    axes[0].axhline(0, color="gray", lw=.8)
    axes[0].set(ylabel="Lateral position (mm)", title="Leader 1.5-wave run: camera versus onboard pose")
    axes[0].legend(ncol=2, fontsize=8)
    axes[1].plot(dense_x, dense_theory, color="#555555", linestyle="--", label="Planned")
    axes[1].plot(odom_x, odom_y, color="#c05a16", label="Nano pose ×1.1483")
    axes[1].plot(vx, video_centered, color="#1865a7", label="Video minus fitted drift axis")
    axes[1].set(xlabel="Forward displacement from start (video axle proxy / onboard MPU, mm)",
                ylabel="Lateral from fitted axis (mm)")
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=.2)
        ax.set_xlim(0, 2020)
    fig.savefig(HERE / "video_vs_odom_path.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.5), constrained_layout=True)
    ax.plot(VIDEO_START_S + elapsed, odom_x, color="#c05a16", label="Onboard x ×1.1483")
    ax.scatter(WHEEL_CONTACT_VIDEO_S, ground_x, color="#1865a7", s=45,
               label="Video wheel-contact x from floor ticks")
    ax.axhline(2000, color="#555555", ls="--", lw=1, label="2 m target")
    ax.set(xlabel="Video time (s)", ylabel="Forward displacement (mm)",
           title="Independent ground-mark checks of forward progress")
    ax.legend(fontsize=8)
    ax.grid(alpha=.2)
    fig.savefig(HERE / "ground_progress_check.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
