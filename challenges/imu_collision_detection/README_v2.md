# IMU Collision Detection

## Approach
Calibrate a baseline acceleration magnitude from the first 15 quiescent IMU samples, then detect collisions using a z-score threshold on incoming samples. A double gate — accel z-score >6σ with a simultaneous gyro spike, or accel alone >8σ — separates true impacts from terrain bumps and vibration. Once a collision is latched, the decision holds for the rest of the episode.

## Major Equations
Acceleration and gyro magnitudes:
```math
a = \sqrt{a_x^2 + a_y^2 + a_z^2}, \quad \omega = \sqrt{g_x^2 + g_y^2 + g_z^2}
```

Baseline statistics from calibration window:
```math
\mu = \frac{1}{N}\sum a_i, \quad \sigma^2 = \frac{1}{N}\sum(a_i - \mu)^2
```

Z-score threshold for collision:
```math
z = \frac{a - \mu}{\sigma} > 8 \quad \text{or} \quad z > 6 \text{ and } \omega > 2.5
```

## Inputs and Expected Output
- **Inputs**: `imu()` samples — linear acceleration $(a_x, a_y, a_z)$ and angular velocity $(g_x, g_y, g_z)$.
- **Output**: `submit_collision_decision(contact, severity)` — severity is `"none"`, `"light"`, or `"hard"`.

## Score obtained with this approach

100 / 100
