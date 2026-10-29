# laser project instructions

Read the repository root `AGENTS.md` for current three-rover setup and operational context. For trajectory figures, follow `pi_rover/PLOT_CONVENTIONS.md`.

- Treat each logged `(x_mm, y_mm)` as the user-defined MPU6050 coordinate. Keep that point visible; do not shift the logged trajectory when drawing the vehicle.
- Draw the 250 mm × 150 mm body around that point using logged heading: nose +100 mm, tail −150 mm, and left/right ±75 mm. Assume the MPU is on the lateral centerline until measured otherwise.
- Show a sparse, translucent dashed body outline and a clear front-direction mark alongside the coordinate point. Preserve equal x/y scale and mm axis labels.
- Label CSV paths as onboard odometry. A body outline reconstructed from CSV is not camera-measured ground truth.
