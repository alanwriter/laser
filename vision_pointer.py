"""Point a two-axis laser mount at the centre of a green X.

Only the green X is searched globally.  Any red-laser detection is strictly
masked to a paper ROI derived from the green X bounding box, so red UI and
objects elsewhere in the camera frame cannot affect calibration or aiming.

For the most robust setup, use --manual-capture: at each of the five supplied
P/T reference poses, click the actual beam hit point yourself.  Those five
measurements fit a full 2-D homography, including the mount's diagonal/cross
axis coupling.
"""

from __future__ import annotations

import argparse
import errno
import json
import math
import re
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np
import serial


WINDOW_NAME = "Green X laser pointer"
Point = tuple[float, float]
Roi = tuple[int, int, int, int]


@dataclass(frozen=True)
class Pose:
    label: str
    pan: float
    tilt: float


@dataclass(frozen=True)
class Target:
    center: Point
    bbox: tuple[int, int, int, int]


@dataclass
class Observation:
    frame: np.ndarray
    target: Target | None
    paper_roi: Roi | None
    red: Point | None


@dataclass
class VideoRecorder:
    path: Path
    writer: cv2.VideoWriter
    frames: int = 0

    def write(self, frame: np.ndarray) -> None:
        self.writer.write(frame)
        self.frames += 1

    def close(self) -> None:
        self.writer.release()
        print(f"Saved recording ({self.frames} frames): {self.path}")


DEFAULT_POSES: tuple[Pose, ...] = (
    Pose("left_top", 155, 30),
    Pose("top_middle", 20, 45),
    Pose("right_bottom", 35, 5),
    Pose("left_bottom", 145, 0),
    Pose("right_top", 5, 35),
)


class UserAbort(RuntimeError):
    """Raised when q is pressed in the OpenCV window."""


class SkipReference(RuntimeError):
    """Raised when a manual calibration reference is visibly outside the paper."""


class DetectionError(RuntimeError):
    """Raised when a required visual feature is unavailable."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Map five P/T reference points to a green X centre with a 2-D coupled transform."
    )
    parser.add_argument(
        "--port",
        default="/dev/cu.usbserial-1320",
        help="Nano serial device (default: /dev/cu.usbserial-1320)",
    )
    parser.add_argument("--camera", type=int, default=0, help="USB camera index (use --list-cameras first)")
    parser.add_argument("--camera-width", type=int, default=1280, help="requested USB camera width")
    parser.add_argument("--camera-height", type=int, default=720, help="requested USB camera height")
    parser.add_argument("--camera-fps", type=float, default=30.0, help="requested USB camera frame rate")
    parser.add_argument(
        "--camera-buffer-size",
        type=int,
        default=1,
        help="requested capture-buffer size; 1 minimizes live-view latency",
    )
    parser.add_argument(
        "--no-record",
        dest="record",
        action="store_false",
        default=True,
        help="do not record annotated video after Space starts P/T movement",
    )
    parser.add_argument(
        "--recording-dir",
        type=Path,
        default=Path("recordings"),
        help="directory for MP4 recordings started by Space",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--serial-ready-timeout",
        type=float,
        default=10.0,
        help="seconds to wait if the Nano USB serial device is re-enumerating",
    )
    parser.add_argument(
        "--serial-ack-timeout",
        type=float,
        default=0.25,
        help="seconds to wait for an optional Nano OK reply; 0 disables waiting",
    )
    parser.add_argument(
        "--mode",
        choices=("preview", "recover", "calibrate", "aim", "calibrate-and-aim"),
        default="calibrate-and-aim",
        help="default: measure five references, then send the calculated centre P/T",
    )
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="list accessible camera indexes and exit without opening Nano serial",
    )
    parser.add_argument("--camera-scan-max", type=int, default=5)
    parser.add_argument(
        "--calibration-file",
        type=Path,
        default=Path("anchor_calibration.json"),
        help="file containing the five P/T-to-pixel reference measurements",
    )
    parser.add_argument(
        "--poses",
        help=(
            "semicolon-separated P/T references. Default: "
            "left_top:155,30;top_middle:20,45;right_bottom:35,5;"
            "left_bottom:145,0;right_top:5,35"
        ),
    )
    parser.add_argument("--pan-min", type=float, default=0)
    parser.add_argument("--pan-max", type=float, default=160)
    parser.add_argument("--tilt-min", type=float, default=0)
    parser.add_argument("--tilt-max", type=float, default=45)
    parser.add_argument("--settle", type=float, default=0.8, help="seconds after each P/T move")
    parser.add_argument(
        "--camera-ready-timeout",
        type=float,
        default=15.0,
        help="seconds to wait for a live camera frame and green X before moving the mount",
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="start P/T movement as soon as camera/X are ready instead of waiting for Space",
    )
    parser.add_argument(
        "--recover-cycles",
        type=int,
        default=1,
        help="how many times recover mode visits the five supplied P/T poses",
    )
    parser.add_argument("--sample-frames", type=int, default=7)
    parser.add_argument("--observation-timeout", type=float, default=4.0)
    parser.add_argument("--click-timeout", type=float, default=90.0)
    parser.add_argument(
        "--manual-capture",
        action="store_true",
        help="always click each actual beam point; no red-colour detection during calibration",
    )
    parser.add_argument(
        "--no-red-feedback",
        action="store_true",
        help="after calculating the centre P/T, send it once without red-dot verification/correction",
    )
    parser.add_argument(
        "--show-paper-red",
        action="store_true",
        help="show only red candidates inside PAPER ROI (display aid; never accepts a manual click automatically)",
    )
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument(
        "--window-x",
        type=int,
        default=80,
        help="OpenCV window X position on the main display",
    )
    parser.add_argument(
        "--window-y",
        type=int,
        default=70,
        help="OpenCV window Y position on the main display",
    )
    parser.add_argument("--window-width", type=int, default=1200, help="display width for the camera window")
    parser.add_argument("--window-height", type=int, default=750, help="display height for the camera window")
    parser.add_argument("--green-hue-low", type=int, default=25)
    parser.add_argument("--green-hue-high", type=int, default=95)
    parser.add_argument("--min-green-area", type=float, default=1200)
    parser.add_argument(
        "--paper-roi-scale",
        type=float,
        default=2.6,
        help="paper ROI size relative to the green X bounding box",
    )
    parser.add_argument("--max-red-area", type=float, default=900)
    parser.add_argument(
        "--min-red-excess",
        type=int,
        default=30,
        help="minimum R-G and R-B difference for a pale red laser spot",
    )
    parser.add_argument("--min-red-value", type=int, default=180, help="minimum red-channel value for a pale red laser spot")
    parser.add_argument(
        "--detect-pale-red",
        action="store_true",
        help="also accept pale pink/overexposed red spots inside PAPER ROI",
    )
    parser.add_argument("--max-detection-spread-px", type=float, default=10.0)
    parser.add_argument(
        "--min-reference-separation-px",
        type=float,
        default=35.0,
        help="minimum pixel separation required between distinct calibration beam points",
    )
    parser.add_argument("--max-reprojection-error-px", type=float, default=35.0)
    parser.add_argument("--calibration-margin-px", type=float, default=0.0)
    parser.add_argument("--deadband-px", type=float, default=12.0)
    parser.add_argument("--max-iterations", type=int, default=8)
    parser.add_argument("--gain", type=float, default=0.70)
    parser.add_argument(
        "--min-step-deg",
        type=float,
        default=1.0,
        help="smallest commanded P/T correction; avoids integer-servo rounding dead zones",
    )
    parser.add_argument("--max-step-deg", type=float, default=2.0)
    parser.add_argument("--max-jacobian-condition", type=float, default=30.0)
    parser.add_argument("--min-jacobian-px-per-deg", type=float, default=0.5)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.pan_min >= args.pan_max or args.tilt_min >= args.tilt_max:
        raise ValueError("Each minimum P/T limit must be smaller than its maximum.")
    if args.camera < 0 or args.camera_scan_max < 0 or args.serial_ack_timeout < 0:
        raise ValueError("camera indexes and serial-ack-timeout cannot be negative.")
    if args.serial_ready_timeout <= 0:
        raise ValueError("serial-ready-timeout must be positive.")
    if args.camera_width < 160 or args.camera_height < 120 or args.camera_fps <= 0 or args.camera_buffer_size < 1:
        raise ValueError("Camera width/height/fps must be positive usable values.")
    if args.window_width < 100 or args.window_height < 100:
        raise ValueError("Camera-window width and height must each be at least 100 pixels.")
    if (
        args.settle < 0
        or args.recover_cycles < 1
        or args.camera_ready_timeout <= 0
        or args.sample_frames < 1
        or args.observation_timeout <= 0
        or args.click_timeout <= 0
    ):
        raise ValueError("Invalid timing or sample-frame value.")
    if args.mode in {"calibrate", "calibrate-and-aim"} and args.no_display:
        raise ValueError("Calibration needs the camera window for paper-ROI confirmation/clicks; remove --no-display.")
    if args.mode == "preview" and args.no_display:
        raise ValueError("preview mode needs the camera window; remove --no-display.")
    if (
        args.paper_roi_scale < 1.0
        or args.min_green_area <= 0
        or args.max_red_area <= 0
        or args.min_red_excess < 1
        or not 1 <= args.min_red_value <= 255
        or args.max_detection_spread_px <= 0
        or args.min_reference_separation_px <= 0
        or args.max_reprojection_error_px <= 0
        or args.calibration_margin_px < 0
        or args.deadband_px < 0
        or args.max_iterations < 1
        or args.gain <= 0
        or args.min_step_deg <= 0
        or args.max_step_deg <= 0
        or args.min_step_deg > args.max_step_deg
        or args.max_jacobian_condition <= 1
        or args.min_jacobian_px_per_deg <= 0
    ):
        raise ValueError("Invalid detection or control parameter.")


def parse_poses(text: str | None) -> list[Pose]:
    if not text:
        return list(DEFAULT_POSES)

    poses: list[Pose] = []
    normalized = text.replace("；", ";").replace("：", ":")
    for index, raw_pose in enumerate(normalized.split(";"), start=1):
        raw_pose = raw_pose.strip()
        if not raw_pose:
            continue
        if ":" in raw_pose:
            label, coordinates = raw_pose.split(":", 1)
            label = label.strip() or f"pose_{index}"
        else:
            label, coordinates = f"pose_{index}", raw_pose
        values = [value.strip() for value in coordinates.replace("，", ",").split(",")]
        if len(values) != 2:
            raise ValueError(f"Invalid pose '{raw_pose}'. Use label:pan,tilt.")
        try:
            pan, tilt = map(float, values)
        except ValueError as error:
            raise ValueError(f"Invalid P/T values in '{raw_pose}'.") from error
        poses.append(Pose(label, pan, tilt))
    if len(poses) < 4:
        raise ValueError("At least four spread-out reference poses are required.")
    return poses


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def pose_in_bounds(pose: Sequence[float], args: argparse.Namespace, margin: float = 0.0) -> bool:
    return (
        args.pan_min - margin <= pose[0] <= args.pan_max + margin
        and args.tilt_min - margin <= pose[1] <= args.tilt_max + margin
    )


class NanoController:
    """Send P,T lines to existing Nano firmware without requiring a particular ACK."""

    _OK = re.compile(r"^OK,P=(-?\d+),T=(-?\d+)$")
    _LEGACY_OK = re.compile(r"^OK,(-?\d+),(-?\d+)$")

    def __init__(self, device: serial.Serial, ack_timeout: float) -> None:
        self.device = device
        self.ack_timeout = ack_timeout

    def move(self, pan: float, tilt: float) -> tuple[float, float]:
        requested = (float(round(pan)), float(round(tilt)))
        command = f"{requested[0]:.0f},{requested[1]:.0f}\n"
        self.device.reset_input_buffer()
        self.device.write(command.encode("ascii"))
        self.device.flush()

        deadline = time.monotonic() + self.ack_timeout
        while time.monotonic() < deadline:
            reply = self.device.readline().decode("ascii", errors="replace").strip()
            if not reply or reply == "READY":
                continue
            match = self._OK.fullmatch(reply) or self._LEGACY_OK.fullmatch(reply)
            if match:
                return float(match.group(1)), float(match.group(2))
            if reply.startswith("ERR"):
                raise RuntimeError(f"Nano rejected '{command.strip()}': {reply}")
        return requested


def open_nano_serial(args: argparse.Namespace) -> serial.Serial:
    """Wait briefly for a Nano that has just re-enumerated on USB."""
    deadline = time.monotonic() + args.serial_ready_timeout
    waiting_reported = False
    last_error: serial.SerialException | None = None
    while time.monotonic() < deadline:
        try:
            return serial.Serial(args.port, args.baud, timeout=0.2)
        except serial.SerialException as error:
            if getattr(error, "errno", None) != errno.ENOENT:
                raise
            last_error = error
            if not waiting_reported:
                print(f"Waiting for Nano serial device {args.port} to reappear...")
                waiting_reported = True
            time.sleep(0.25)
    assert last_error is not None
    raise last_error


def open_camera(
    index: int,
    width: int | None = None,
    height: int | None = None,
    fps: float | None = None,
    buffer_size: int | None = None,
) -> cv2.VideoCapture:
    if sys.platform == "darwin" and hasattr(cv2, "CAP_AVFOUNDATION"):
        cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    else:
        cap = cv2.VideoCapture(index)
    if cap.isOpened():
        if buffer_size is not None:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)
        if width is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height is not None:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps is not None:
            cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def start_recording(cap: cv2.VideoCapture, args: argparse.Namespace) -> VideoRecorder:
    """Create an annotated MP4 writer after the user has pressed Space."""
    width = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH))) or args.camera_width
    height = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))) or args.camera_height
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    if not np.isfinite(fps) or fps < 1.0:
        fps = args.camera_fps
    args.recording_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = args.recording_dir / f"laser-{stamp}.mp4"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError("Could not create MP4 recording. Try --no-record and check the recordings directory permissions.")
    recorder = VideoRecorder(path, writer)
    print(f"Recording started: {path}")
    return recorder


def read_camera_frame(cap: cv2.VideoCapture, timeout: float = 2.0) -> np.ndarray:
    """Allow an AVFoundation USB camera a short warm-up before failing."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ok, frame = cap.read()
        if ok and frame is not None:
            return frame
        time.sleep(0.05)
    raise RuntimeError("USB camera frame read failed after waiting for camera warm-up.")


def list_cameras(max_index: int) -> None:
    print("Accessible camera indexes:")
    found = False
    for index in range(max_index + 1):
        cap = open_camera(index)
        try:
            if cap.isOpened():
                try:
                    read_camera_frame(cap, timeout=0.8)
                except RuntimeError:
                    pass
                else:
                    print(f"  {index}")
                    found = True
        finally:
            cap.release()
    if not found:
        print("  none — grant Camera permission to this Terminal/IDE, then retry.")


def wait_for_camera_ready(cap: cv2.VideoCapture, args: argparse.Namespace) -> cv2.VideoCapture:
    """Show a live, correctly located target before any Nano command is sent."""
    print("Waiting for live USB camera and GREEN X before moving the mount... (q=stop)")
    deadline = time.monotonic() + args.camera_ready_timeout
    current = cap
    ready_announced = False
    while ready_announced or time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            frame = read_camera_frame(current, timeout=min(2.0, max(0.1, remaining)))
        except RuntimeError:
            # A just-closed AVFoundation camera may need to be opened again.
            current.release()
            time.sleep(0.25)
            current = open_camera(
                args.camera,
                args.camera_width,
                args.camera_height,
                args.camera_fps,
                args.camera_buffer_size,
            )
            continue
        target = find_green_x(frame, args)
        roi = paper_roi(target, frame.shape, args.paper_roi_scale) if target else None
        key = show_frame(
            frame,
            args,
            target,
            roi,
            None,
            (
                "Camera ready: Space=start P/T movement / q=stop"
                if target is not None
                else "Waiting for live GREEN X before moving mount / q=stop"
            ),
        )
        if key == ord("q"):
            raise UserAbort("Stopped by user.")
        if target is not None:
            if not ready_announced:
                print(f"Camera stream: {frame.shape[1]}x{frame.shape[0]}.")
                print("Camera and GREEN X are ready. Press Space in the camera window to start P/T movement.")
                ready_announced = True
            if args.no_display or args.auto_start or key == ord(" "):
                print("Starting P/T movement.")
                return current
    current.release()
    raise RuntimeError(
        f"USB camera/green X was not ready within {args.camera_ready_timeout:g} s; no P/T command was sent."
    )


def green_mask(frame: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array((args.green_hue_low, 50, 35), dtype=np.uint8),
        np.array((args.green_hue_high, 255, 255), dtype=np.uint8),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)),
        iterations=2,
    )
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        iterations=1,
    )


def find_green_x(frame: np.ndarray, args: argparse.Namespace) -> Target | None:
    contours, _ = cv2.findContours(green_mask(frame, args), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    frame_height, frame_width = frame.shape[:2]
    image_center = np.asarray((frame_width / 2.0, frame_height / 2.0), dtype=np.float64)
    # The printed X is the large green object near the centre of the camera
    # view.  In low light an unrelated green object can be brighter than it;
    # rank by both size and centrality instead of simply taking the largest.
    distance_scale = max(1.0, min(frame_width, frame_height) * 0.22)
    candidates: list[tuple[float, np.ndarray]] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < args.min_green_area:
            continue
        centre, _, _ = cv2.minAreaRect(contour)
        distance = float(np.linalg.norm(np.asarray(centre) - image_center))
        score = area / (1.0 + (distance / distance_scale) ** 2)
        candidates.append((score, contour))
    if not candidates:
        return None
    contour = max(candidates, key=lambda candidate: candidate[0])[1]
    center, _, _ = cv2.minAreaRect(contour)
    x, y, width, height = cv2.boundingRect(contour)
    return Target((float(center[0]), float(center[1])), (x, y, width, height))


def paper_roi(target: Target, frame_shape: Sequence[int], scale: float) -> Roi:
    """A conservative white-paper search area inferred from the green X bounds."""
    height, width = int(frame_shape[0]), int(frame_shape[1])
    x, y, box_width, box_height = target.bbox
    cx, cy = x + box_width / 2.0, y + box_height / 2.0
    roi_width, roi_height = box_width * scale, box_height * scale
    left = max(0, int(round(cx - roi_width / 2.0)))
    top = max(0, int(round(cy - roi_height / 2.0)))
    right = min(width, int(round(cx + roi_width / 2.0)))
    bottom = min(height, int(round(cy + roi_height / 2.0)))
    return left, top, right, bottom


def red_mask_inside_paper(frame: np.ndarray, args: argparse.Namespace, roi: Roi) -> np.ndarray:
    """Return strongly red beam candidates only inside the target-derived paper ROI."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(hsv, np.array((0, 110, 145), np.uint8), np.array((12, 255, 255), np.uint8))
    high_red = cv2.inRange(hsv, np.array((168, 110, 145), np.uint8), np.array((179, 255, 255), np.uint8))
    blue, green, red = cv2.split(frame)
    blue16 = blue.astype(np.int16)
    green16 = green.astype(np.int16)
    red16 = red.astype(np.int16)
    red_dominant = (
        (red16 > 145)
        & (red16 > green16 * 1.28)
        & (red16 > blue16 * 1.28)
    ).astype(np.uint8) * 255
    # Laser dots on white paper are sometimes pale pink/overexposed.  This
    # optional branch is deliberately off by default: white-paper texture can
    # otherwise look mildly red in a JPEG/webcam frame.
    pale_red = (
        (red16 >= args.min_red_value)
        & (red16 - green16 >= args.min_red_excess)
        & (red16 - blue16 >= args.min_red_excess)
    ).astype(np.uint8) * 255
    # Do not treat yellow/green highlights as a laser.  A genuine red dot on
    # the green print remains either red-hued or strongly red-channel dominant.
    mask = cv2.bitwise_or(low_red, high_red)
    mask = cv2.bitwise_or(mask, red_dominant)
    if args.detect_pale_red:
        mask = cv2.bitwise_or(mask, pale_red)
    left, top, right, bottom = roi
    paper_mask = np.zeros(mask.shape, dtype=np.uint8)
    paper_mask[top:bottom, left:right] = 255
    mask = cv2.bitwise_and(mask, paper_mask)
    # Closing preserves a one-pixel/very small laser hit; opening would erase
    # it before temporal stability can reject transient sensor noise.
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    )


def find_red_laser(frame: np.ndarray, args: argparse.Namespace, roi: Roi, expected: Point | None = None) -> Point | None:
    mask = red_mask_inside_paper(frame, args, roi)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[tuple[float, Point]] = []
    for contour in contours:
        moments = cv2.moments(contour)
        if moments["m00"]:
            point = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        else:
            x, y, width, height = cv2.boundingRect(contour)
            point = (x + width / 2.0, y + height / 2.0)
        x, y, width, height = cv2.boundingRect(contour)
        pixel_area = float(cv2.countNonZero(mask[y : y + height, x : x + width]))
        if pixel_area < 1.0 or pixel_area > args.max_red_area:
            continue
        mean_bgr = cv2.mean(frame[y : y + height, x : x + width])[:3]
        redness = mean_bgr[2] - max(mean_bgr[0], mean_bgr[1])
        if expected is None:
            (_, _), radius = cv2.minEnclosingCircle(contour)
            compactness = min(1.0, pixel_area / max(math.pi * radius * radius, 1.0))
            score = redness + 80.0 * compactness
        else:
            score = 80.0 + redness * 0.08 - math.dist(point, expected)
        candidates.append((score, (float(point[0]), float(point[1]))))
    return max(candidates, key=lambda item: item[0])[1] if candidates else None


def median_point(points: Iterable[Point]) -> Point:
    values = np.asarray(list(points), dtype=np.float64)
    return float(np.median(values[:, 0])), float(np.median(values[:, 1]))


def stable_point(points: Sequence[Point], max_spread: float) -> Point | None:
    if not points:
        return None
    centre = median_point(points)
    spread = max(math.dist(point, centre) for point in points)
    return centre if spread <= max_spread else None


def draw_point(
    frame: np.ndarray,
    point: Point | None,
    color: tuple[int, int, int],
    label: str,
    *,
    draw_cross: bool = True,
) -> None:
    if point is None:
        return
    x, y = round(point[0]), round(point[1])
    cv2.circle(frame, (x, y), 12, color, 2)
    if draw_cross:
        cv2.drawMarker(frame, (x, y), color, cv2.MARKER_CROSS, 20, 2)
    cv2.putText(frame, label, (x + 14, y - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


def show_frame(
    frame: np.ndarray,
    args: argparse.Namespace,
    target: Target | None,
    roi: Roi | None,
    red: Point | None,
    status: str,
    clicked: Point | None = None,
) -> int:
    if args.no_display:
        return -1
    preview = frame.copy()
    if roi is not None:
        left, top, right, bottom = roi
        cv2.rectangle(preview, (left, top), (right - 1, bottom - 1), (0, 255, 255), 2)
        cv2.putText(preview, "PAPER ROI", (left + 8, top + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    draw_point(preview, target.center if target else None, (0, 255, 0), "GREEN X")
    draw_point(preview, red, (0, 0, 255), "LASER (paper only)", draw_cross=False)
    draw_point(preview, clicked, (255, 255, 0), "REFERENCE")
    cv2.putText(preview, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    recorder: VideoRecorder | None = getattr(args, "_recorder", None)
    if recorder is not None:
        recorder.write(preview)
    cv2.imshow(WINDOW_NAME, preview)
    key = cv2.waitKey(1) & 0xFF
    # macOS creates an OpenCV native window lazily, after its first event-loop
    # turn.  Reapply placement for the first two seconds so it does not remain
    # on the secondary display when the first move is ignored.
    placement_attempts = getattr(args, "_window_placement_attempts", 0)
    if placement_attempts < 60:
        cv2.resizeWindow(WINDOW_NAME, args.window_width, args.window_height)
        cv2.moveWindow(WINDOW_NAME, args.window_x, args.window_y)
        args._window_placement_attempts = placement_attempts + 1
    return key


def wait_with_live_preview(cap: cv2.VideoCapture, args: argparse.Namespace, seconds: float, status: str) -> None:
    """Wait for the mount while continuously servicing the camera window."""
    if seconds <= 0:
        return
    if args.no_display:
        time.sleep(seconds)
        return

    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        frame = read_camera_frame(cap, timeout=min(1.0, max(0.1, remaining)))
        target = find_green_x(frame, args)
        roi = paper_roi(target, frame.shape, args.paper_roi_scale) if target else None
        key = show_frame(frame, args, target, roi, None, status)
        if key == ord("q"):
            raise UserAbort


def read_observation(
    cap: cv2.VideoCapture,
    args: argparse.Namespace,
    *,
    require_red: bool,
    expected_red: Point | None = None,
    status: str,
) -> Observation:
    targets: deque[Target] = deque(maxlen=args.sample_frames)
    reds: deque[Point] = deque(maxlen=args.sample_frames)
    latest_frame: np.ndarray | None = None
    deadline = time.monotonic() + args.observation_timeout

    while time.monotonic() < deadline:
        frame = read_camera_frame(cap)
        latest_frame = frame
        target = find_green_x(frame, args)
        roi = paper_roi(target, frame.shape, args.paper_roi_scale) if target else None
        # Do not even run colour-based red detection unless this caller needs
        # it.  When it is needed, find_red_laser is still hard-masked to ROI.
        red = find_red_laser(frame, args, roi, expected_red) if require_red and roi else None

        if target is None:
            targets.clear()
            reds.clear()
        else:
            targets.append(target)
            if red is not None:
                reds.append(red)
            elif require_red:
                reds.clear()

        key = show_frame(frame, args, target, roi, red, status)
        if key == ord("q"):
            raise UserAbort("Stopped by user.")

        target_points = [value.center for value in targets]
        stable_target = (
            stable_point(target_points, args.max_detection_spread_px)
            if len(targets) == args.sample_frames
            else None
        )
        stable_red = stable_point(list(reds), args.max_detection_spread_px) if len(reds) == args.sample_frames else None
        if stable_target is not None and (not require_red or stable_red is not None):
            # The latest target bbox makes the ROI follow any small camera jitter.
            current_target = targets[-1]
            current_roi = paper_roi(current_target, latest_frame.shape, args.paper_roi_scale)
            return Observation(
                latest_frame,
                Target(stable_target, current_target.bbox),
                current_roi,
                stable_red,
            )

    missing = "green X" if not targets else "stable red laser inside PAPER ROI"
    raise DetectionError(f"Could not find {missing} within {args.observation_timeout:g} s.")


def point_inside_roi(point: Point, roi: Roi) -> bool:
    left, top, right, bottom = roi
    return left <= point[0] < right and top <= point[1] < bottom


def click_reference_point(cap: cv2.VideoCapture, args: argparse.Namespace, label: str, pose: Pose) -> tuple[Point, tuple[int, int]]:
    """User selects the real beam point; no red-colour detector is involved."""
    clicked: list[Point] = []
    active_roi: list[Roi | None] = [None]

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            point = (float(x), float(y))
            if active_roi[0] is not None and point_inside_roi(point, active_roi[0]):
                clicked[:] = [point]
            else:
                print("Click ignored: choose the beam point inside the yellow PAPER ROI.")

    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    print(
        f"{label}: click the actual beam hit point inside the yellow PAPER ROI "
        f"(P={pose.pan:g}, T={pose.tilt:g}); press n to try the next reference."
    )
    deadline = time.monotonic() + args.click_timeout
    while time.monotonic() < deadline:
        frame = read_camera_frame(cap)
        target = find_green_x(frame, args)
        roi = paper_roi(target, frame.shape, args.paper_roi_scale) if target else None
        active_roi[0] = roi
        # This is display-only assistance.  Manual capture still records only
        # the user's click, so a false red candidate cannot corrupt calibration.
        red_hint = find_red_laser(frame, args, roi) if args.show_paper_red and roi else None
        key = show_frame(
            frame,
            args,
            target,
            roi,
            red_hint,
            f"{label}: click actual beam in yellow ROI / n=next / q=stop",
            clicked[0] if clicked else None,
        )
        if clicked:
            return clicked[0], (int(frame.shape[1]), int(frame.shape[0]))
        if key == ord("q"):
            raise UserAbort("Stopped by user.")
        if key == ord("n"):
            raise SkipReference(f"Skipped {label}: beam was not inside the yellow PAPER ROI.")
    raise DetectionError(f"Timed out waiting for a reference click at {label}.")


def capture_reference_point(cap: cv2.VideoCapture, args: argparse.Namespace, label: str, pose: Pose) -> tuple[Point, tuple[int, int]]:
    if args.manual_capture:
        return click_reference_point(cap, args, label, pose)
    try:
        observation = read_observation(
            cap,
            args,
            require_red=True,
            status=f"{label}: searching red only inside yellow PAPER ROI",
        )
        assert observation.red is not None
        return observation.red, (int(observation.frame.shape[1]), int(observation.frame.shape[0]))
    except DetectionError:
        print(f"No reliable in-paper red point at {label}; switching to manual click (no red detection).")
        return click_reference_point(cap, args, label, pose)


def transform_points(matrix: np.ndarray, points: Sequence[Sequence[float]]) -> np.ndarray:
    source = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(source, matrix).reshape(-1, 2)


def project_pose(matrix: np.ndarray, pose: Sequence[float]) -> Point:
    point = transform_points(matrix, [pose])[0]
    return float(point[0]), float(point[1])


def inverse_project(matrix: np.ndarray, point: Point) -> tuple[float, float]:
    inverse = np.linalg.inv(matrix)
    pose = project_pose(inverse, point)
    if not np.isfinite(pose).all():
        raise RuntimeError("The green X centre maps to an invalid P/T pose.")
    return pose


def fit_homography(samples: Sequence[dict[str, float | str]]) -> tuple[np.ndarray, np.ndarray]:
    if len(samples) < 3:
        raise ValueError("Need at least three P/T-to-pixel references.")
    servo_points = np.asarray([[sample["pan"], sample["tilt"]] for sample in samples], dtype=np.float64)
    pixels = np.asarray([[sample["x"], sample["y"]] for sample in samples], dtype=np.float64)
    if len(samples) == 3:
        # Three non-collinear references determine a full 2-D affine map.  It
        # retains pan/tilt cross-coupling and skew, but deliberately does not
        # claim to model perspective as a five-point homography does.
        affine = cv2.getAffineTransform(servo_points.astype(np.float32), pixels.astype(np.float32))
        matrix = np.vstack((affine, (0.0, 0.0, 1.0)))
    else:
        matrix, _ = cv2.findHomography(servo_points, pixels, method=0)
    if matrix is None:
        raise RuntimeError("Reference poses are degenerate; use spread-out beam points.")
    errors = np.linalg.norm(transform_points(matrix, servo_points) - pixels, axis=1)
    return matrix, errors


def validate_reference_pixels(points: np.ndarray, args: argparse.Namespace) -> None:
    """Reject a detector that has repeatedly latched on to one static artifact."""
    distinct: list[np.ndarray] = []
    for point in points:
        if all(float(np.linalg.norm(point - previous)) >= args.min_reference_separation_px for previous in distinct):
            distinct.append(point)
    if len(distinct) < 3:
        raise RuntimeError(
            f"Only {len(distinct)} distinct beam locations were measured; need at least three. "
            "The red detector likely followed a fixed reflection. Re-run with --manual-capture and click the real dot."
        )


def save_calibration(path: Path, matrix: np.ndarray, samples: Sequence[dict[str, float | str]], frame_size: tuple[int, int], errors: np.ndarray) -> None:
    payload = {
        "version": 3,
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "frame_size": list(frame_size),
        "servo_to_pixel_homography": matrix.tolist(),
        "transform_model": "affine_3_point" if len(samples) == 3 else "homography",
        "samples": list(samples),
        "reprojection_errors_px": [float(error) for error in errors],
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def load_calibration(path: Path, args: argparse.Namespace) -> tuple[np.ndarray, tuple[int, int], np.ndarray]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        matrix = np.asarray(payload["servo_to_pixel_homography"], dtype=np.float64)
        frame_size = tuple(map(int, payload["frame_size"]))
        footprint = np.asarray([[item["x"], item["y"]] for item in payload["samples"]], dtype=np.float64)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Cannot read '{path}'. Run --mode calibrate first.") from error
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or abs(np.linalg.det(matrix)) < 1e-10:
        raise RuntimeError("Calibration contains an invalid 2-D transformation matrix.")
    if len(frame_size) != 2 or footprint.shape[0] < 3 or footprint.shape[1:] != (2,):
        raise RuntimeError("Calibration does not contain enough valid reference points.")
    validate_reference_pixels(footprint, args)
    return matrix, (frame_size[0], frame_size[1]), footprint


def ensure_resolution(frame: np.ndarray, expected: tuple[int, int]) -> None:
    height, width = frame.shape[:2]
    if (width, height) != expected:
        raise RuntimeError(f"Camera resolution changed from {expected} to {(width, height)}; re-calibrate.")


def ensure_inside_footprint(target: Point, footprint: np.ndarray, args: argparse.Namespace) -> None:
    hull = cv2.convexHull(footprint.astype(np.float32))
    distance = float(cv2.pointPolygonTest(hull, target, True))
    if distance < -args.calibration_margin_px:
        raise RuntimeError(
            f"Green X centre is {abs(distance):.0f}px outside the five clicked reference points. "
            "Re-calibrate with wider points."
        )


def homography_jacobian(matrix: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    base = np.asarray(project_pose(matrix, pose), dtype=np.float64)
    pan_probe = np.asarray(project_pose(matrix, (pose[0] + 1.0, pose[1])), dtype=np.float64)
    tilt_probe = np.asarray(project_pose(matrix, (pose[0], pose[1] + 1.0)), dtype=np.float64)
    return np.column_stack((pan_probe - base, tilt_probe - base))


def check_jacobian(jacobian: np.ndarray, args: argparse.Namespace) -> None:
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    condition = float(np.linalg.cond(jacobian))
    smallest = float(singular_values[-1])
    if not np.isfinite(condition) or condition > args.max_jacobian_condition or smallest < args.min_jacobian_px_per_deg:
        raise RuntimeError(
            f"Reference transform is unsafe (condition={condition:.1f}, min={smallest:.2f}px/deg). "
            "Re-capture the five points."
        )


def calibrate(controller: NanoController, cap: cv2.VideoCapture, args: argparse.Namespace, poses: Sequence[Pose]) -> tuple[np.ndarray, tuple[int, int], np.ndarray]:
    samples: list[dict[str, float | str]] = []
    frame_size: tuple[int, int] | None = None
    print("Reference calibration: every automatic red search is restricted to the yellow PAPER ROI.")
    for pose in poses:
        if not pose_in_bounds((pose.pan, pose.tilt), args):
            raise ValueError(f"Reference {pose.label} ({pose.pan:g},{pose.tilt:g}) is outside configured P/T limits.")
        print(f"  {pose.label}: send P={pose.pan:g}, T={pose.tilt:g}")
        actual_pan, actual_tilt = controller.move(pose.pan, pose.tilt)
        if abs(actual_pan - pose.pan) > 0.5 or abs(actual_tilt - pose.tilt) > 0.5:
            print(
                f"    Nano reported P={actual_pan:g}, T={actual_tilt:g}; "
                "the calibration will use these actual values."
            )
        wait_with_live_preview(cap, args, args.settle, f"{pose.label}: mount settling")
        try:
            point, frame_size = capture_reference_point(cap, args, pose.label, pose)
        except SkipReference as error:
            print(f"    {error}")
            continue
        samples.append({"label": pose.label, "pan": actual_pan, "tilt": actual_tilt, "x": point[0], "y": point[1]})
        print(f"    reference pixel = ({point[0]:.1f}, {point[1]:.1f})")

    if frame_size is None:
        raise RuntimeError("No supplied P/T reference put the beam inside the yellow PAPER ROI.")
    if len(samples) < 3:
        raise RuntimeError(
            f"Only {len(samples)} in-paper reference points were captured. "
            "At least three spread-out points are needed; reposition the mount/paper or provide additional P/T poses."
        )
    actual_poses = {(float(sample["pan"]), float(sample["tilt"])) for sample in samples}
    if len(actual_poses) < 3:
        raise RuntimeError(
            "Nano limited the supplied references to fewer than three distinct P/T positions. "
            "Use five positions that the existing Nano firmware can actually reach."
        )
    reference_pixels = np.asarray([[sample["x"], sample["y"]] for sample in samples], dtype=np.float64)
    validate_reference_pixels(reference_pixels, args)
    matrix, errors = fit_homography(samples)
    if len(samples) > 4 and float(np.max(errors)) > args.max_reprojection_error_px:
        raise RuntimeError(
            f"Reference fit error is too large (max {np.max(errors):.1f}px). "
            "Use --manual-capture and click each real beam point."
        )
    save_calibration(args.calibration_file, matrix, samples, frame_size, errors)
    model = "3-point affine fallback" if len(samples) == 3 else "homography"
    print(f"Saved {len(samples)} references to {args.calibration_file} ({model}); max fit error {np.max(errors):.1f}px.")
    footprint = np.asarray([[sample["x"], sample["y"]] for sample in samples], dtype=np.float64)
    return matrix, frame_size, footprint


def aim(controller: NanoController, cap: cv2.VideoCapture, args: argparse.Namespace, matrix: np.ndarray, frame_size: tuple[int, int], footprint: np.ndarray) -> bool:
    first = read_observation(cap, args, require_red=False, status="Finding green X")
    assert first.target is not None
    ensure_resolution(first.frame, frame_size)
    target = first.target.center
    ensure_inside_footprint(target, footprint, args)
    current = inverse_project(matrix, target)
    if not pose_in_bounds(current, args, margin=0.5):
        raise RuntimeError(f"Calculated centre P/T=({current[0]:.1f},{current[1]:.1f}) is outside safe limits.")

    current = controller.move(*current)
    print(f"Calculated centre command: P={current[0]:.0f}, T={current[1]:.0f}")
    wait_with_live_preview(cap, args, args.settle, "Calculated centre P/T: mount settling")
    if args.no_red_feedback:
        print("Sent one calculated P/T command; red feedback is disabled by --no-red-feedback.")
        return True

    for iteration in range(1, args.max_iterations + 1):
        expected = project_pose(matrix, current)
        observation = read_observation(
            cap,
            args,
            require_red=True,
            expected_red=expected,
            status=f"Aiming {iteration}/{args.max_iterations}: red search only inside PAPER ROI",
        )
        assert observation.target is not None and observation.red is not None
        ensure_resolution(observation.frame, frame_size)
        target, beam = observation.target.center, observation.red
        error = np.asarray(target) - np.asarray(beam)
        error_size = float(np.linalg.norm(error))
        print(f"  {iteration}: X=({target[0]:.1f},{target[1]:.1f}) beam=({beam[0]:.1f},{beam[1]:.1f}) error={error_size:.1f}px")
        if error_size <= args.deadband_px:
            print("Success: in-paper beam is at the green X centre.")
            return True
        jacobian = homography_jacobian(matrix, current)
        check_jacobian(jacobian, args)
        correction = np.linalg.pinv(jacobian) @ error * args.gain
        largest = float(np.max(np.abs(correction)))
        if largest > args.max_step_deg:
            correction *= args.max_step_deg / largest
        elif largest > 1e-9 and largest < args.min_step_deg:
            # Nano firmware accepts integer P/T commands.  Without a lower
            # bound, a useful sub-degree visual correction can round back to
            # the same command forever, appearing as a frozen servo.
            correction *= args.min_step_deg / largest
        requested = tuple(np.asarray(current) + correction)
        if not pose_in_bounds(requested, args):
            raise RuntimeError("The next coupled P/T correction would cross a configured limit.")
        print(
            f"    correction dP={correction[0]:+.2f}, dT={correction[1]:+.2f} "
            f"-> send P={round(requested[0])}, T={round(requested[1])}"
        )
        current = controller.move(*requested)
        wait_with_live_preview(cap, args, args.settle, f"Aiming {iteration}: mount settling")

    print("Stopped at iteration limit; inspect the yellow PAPER ROI and rerun --manual-capture if needed.")
    return False


def preview(cap: cv2.VideoCapture, args: argparse.Namespace) -> None:
    print("Preview: q exits. Red candidates are hidden unless --show-paper-red is supplied.")
    while True:
        frame = read_camera_frame(cap)
        target = find_green_x(frame, args)
        roi = paper_roi(target, frame.shape, args.paper_roi_scale) if target else None
        red = find_red_laser(frame, args, roi) if args.show_paper_red and roi else None
        key = show_frame(frame, args, target, roi, red, "Preview — q to exit")
        if key == ord("q"):
            return


def recover_to_paper(
    controller: NanoController,
    cap: cv2.VideoCapture,
    args: argparse.Namespace,
    poses: Sequence[Pose],
) -> bool:
    """Visit known safe poses until the beam is seen inside the paper ROI."""
    print(
        "Recovery: visiting the supplied P/T references. "
        "Only a stable red point inside the yellow PAPER ROI is accepted."
    )
    for cycle in range(1, args.recover_cycles + 1):
        for pose in poses:
            if not pose_in_bounds((pose.pan, pose.tilt), args):
                raise ValueError(f"Reference {pose.label} ({pose.pan:g},{pose.tilt:g}) is outside configured P/T limits.")
            actual = controller.move(pose.pan, pose.tilt)
            print(
                f"  recovery {cycle}/{args.recover_cycles}: {pose.label} "
                f"P={actual[0]:g}, T={actual[1]:g}"
            )
            wait_with_live_preview(cap, args, args.settle, f"Recovery {pose.label}: mount settling")
            try:
                observation = read_observation(
                    cap,
                    args,
                    require_red=True,
                    status=f"Recovery {pose.label}: searching red only inside yellow PAPER ROI / q=stop",
                )
            except DetectionError:
                print("    no in-paper red point at this pose; trying next reference.")
                continue
            assert observation.red is not None
            print(
                f"Recovered at {pose.label}: P={actual[0]:g}, T={actual[1]:g}, "
                f"beam=({observation.red[0]:.1f}, {observation.red[1]:.1f})."
            )
            return True
    return False


def main() -> None:
    args = parse_args()
    validate_args(args)
    if args.list_cameras:
        list_cameras(args.camera_scan_max)
        return

    cap = open_camera(
        args.camera,
        args.camera_width,
        args.camera_height,
        args.camera_fps,
        args.camera_buffer_size,
    )
    if not cap.isOpened():
        raise RuntimeError(
            f"Cannot open USB camera index {args.camera}. Enable Camera permission for this Terminal/IDE, "
            "then run --list-cameras."
        )
    if not args.no_display:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        if args.mode == "preview":
            preview(cap, args)
            return
        cap = wait_for_camera_ready(cap, args)
        if args.record:
            args._recorder = start_recording(cap, args)
        poses = parse_poses(args.poses)
        with open_nano_serial(args) as device:
            # Existing Nano firmware may reset after serial opens; keep the UI alive.
            wait_with_live_preview(cap, args, 2.0, "Nano connected: waiting for reset")
            device.reset_input_buffer()
            controller = NanoController(device, args.serial_ack_timeout)
            if args.mode == "recover":
                if recover_to_paper(controller, cap, args, poses):
                    return
                raise RuntimeError("None of the supplied P/T references produced a red point inside PAPER ROI.")
            if args.mode in {"calibrate", "calibrate-and-aim"}:
                matrix, frame_size, footprint = calibrate(controller, cap, args, poses)
            else:
                matrix, frame_size, footprint = load_calibration(args.calibration_file, args)
            if args.mode == "calibrate":
                return
            if not aim(controller, cap, args, matrix, frame_size, footprint):
                raise RuntimeError("Aiming did not reach the configured pixel deadband.")
    finally:
        recorder: VideoRecorder | None = getattr(args, "_recorder", None)
        if recorder is not None:
            recorder.close()
            args._recorder = None
        cap.release()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except UserAbort as error:
        print(error)
        raise SystemExit(1)
    except (DetectionError, RuntimeError, ValueError, serial.SerialException) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
