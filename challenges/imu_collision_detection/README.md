# IMU Collision Detection

## Approach
This solution uses IMU sensors to detect collisions. This is done by calculating jerk (derivative of acceleration) and angular jerk (derivative of angular velocity). A running variance of both values is maintained and if a combined, normalized 'z-score' (normalized linear jerk plus normalized angular jerk) is greater than a threshold, then a collision has occurred. The theory behind this approach is that a sudden change in the linear acceleration or angular velocity is representative of a collision. The linear jerk is calculated as the derivative of the magnitude of the acceleration vector, the angular jerk as the derivative of the magnitude of the angular velocity vector. Both of these are then divided by the running variance of the corresponding values to get a 'z-score' after which they are added together.

## Major Equations
Jerk is calculated as the rate of change of magnitude:
$$ j_{accel} = \frac{|\vec{a}_t| - |\vec{a}_{t-1}|}{\Delta t} $$

The variance is updated using an exponential moving average:
$$ \sigma_{accel}^2 = 0.98 \cdot \sigma_{accel}^2 + 0.02 \cdot j_{accel}^2 $$

Collision is triggered based on normalized scores:
$$ z_{accel} = \frac{j_{accel}}{\max(0.5, \sigma_{accel})}, \quad z_{gyro} = \frac{j_{gyro}}{\max(0.3, \sigma_{gyro})} $$
$$ \text{Collision if } z_{accel} \cdot z_{gyro} > 12.0 $$

## Inputs and Expected Output
- **Inputs**: IMU state containing 3-axis acceleration (`ax, ay, az`), 3-axis angular velocity (`gx, gy, gz`), and timestamp (`t`).
- **Output**: Submits a boolean decision and severity (`"none"`, `"light"`, `"hard"`) via `submit_collision_decision()`.

## Score obtained with this approach

100 / 100