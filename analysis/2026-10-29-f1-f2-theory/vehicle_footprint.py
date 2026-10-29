"""Draw a rover body around the project's MPU6050 pose reference.

The logged (x, y) is the user-defined MPU6050 position. The nose is 100 mm
forward of it; the tail is 150 mm behind it; body width is 150 mm. The MPU
is assumed to lie on the vehicle's lateral centerline.
"""

from __future__ import annotations

import math

from matplotlib.axes import Axes
from matplotlib.colors import to_rgba
from matplotlib.patches import Polygon

FRONT_MM = 100.0
REAR_MM = 150.0
HALF_WIDTH_MM = 75.0


def body_point(x_mm: float, y_mm: float, heading_deg: float,
               forward_mm: float, left_mm: float) -> tuple[float, float]:
    angle = math.radians(heading_deg)
    return (
        x_mm + forward_mm * math.cos(angle) - left_mm * math.sin(angle),
        y_mm + forward_mm * math.sin(angle) + left_mm * math.cos(angle),
    )


def body_corners(x_mm: float, y_mm: float,
                 heading_deg: float) -> list[tuple[float, float]]:
    """Return nose-left, nose-right, tail-right, tail-left in world mm."""
    return [body_point(x_mm, y_mm, heading_deg, forward, left) for forward, left in (
        (FRONT_MM, HALF_WIDTH_MM),
        (FRONT_MM, -HALF_WIDTH_MM),
        (-REAR_MM, -HALF_WIDTH_MM),
        (-REAR_MM, HALF_WIDTH_MM),
    )]


def draw_body(ax: Axes, x_mm: float, y_mm: float, heading_deg: float,
              color: str, *, prominent: bool = False,
              draw_anchor: bool = True) -> None:
    """Use a translucent body, dashed perimeter, solid nose, and MPU dot."""
    corners = body_corners(x_mm, y_mm, heading_deg)
    ax.add_patch(Polygon(
        corners, closed=True,
        facecolor=to_rgba(color, 0.13 if prominent else 0.055),
        edgecolor=to_rgba(color, 0.85 if prominent else 0.47),
        linewidth=1.55 if prominent else 1.0,
        linestyle=(0, (3, 2)), zorder=6 if prominent else 3,
    ))
    if prominent:
        front_left, front_right = corners[:2]
        ax.plot([front_left[0], front_right[0]],
                [front_left[1], front_right[1]],
                color=color, lw=2.0, zorder=7)
        arrow_end = body_point(x_mm, y_mm, heading_deg, 70, 0)
        ax.annotate("", xy=arrow_end, xytext=(x_mm, y_mm),
                    arrowprops={"arrowstyle": "-|>", "color": color,
                                "lw": 1.3, "mutation_scale": 10},
                    zorder=8)
    if draw_anchor:
        ax.scatter(x_mm, y_mm, s=55 if prominent else 26,
                   facecolor="white", edgecolor=color,
                   linewidth=1.7 if prominent else 1.2, zorder=9)
        ax.scatter(x_mm, y_mm, s=9 if prominent else 5,
                   color=color, zorder=10)
