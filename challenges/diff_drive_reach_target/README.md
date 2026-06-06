# Differential Drive: Reach Target

## Approach
Reactive heading control with lidar-based obstacle avoidance. Each step computes a bearing error to the target and drives wheel velocities using a unicycle model. Forward speed is scaled by angular alignment and proximity to the target. Obstacle repulsion is derived from the lidar scan; close readings in the forward arc brake the robot and inject a steering bias away from the nearest threat.

## Major Equations
Bearing error to target:
```math
e_\theta = \text{wrap}(\text{atan2}(\Delta y, \Delta x) - \theta)
```

Forward velocity scaled by alignment and obstacle proximity:
```math
v = V_{max} \cdot \min\!\left(1, \frac{d}{0.5}\right) \cdot \cos^2(e_\theta) \cdot b_{obs}
```

Obstacle brake and steer from lidar:
```math
b_{obs} = \frac{r_{fwd,min} - r_{hard}}{r_{safe} - r_{hard}}, \quad
\delta_{obs} = -2.2\sum_i \sin(\phi_i)\cdot d_i \cdot \cos^+(\phi_i)
```

Unicycle to differential drive:
```math
v_L = v - \tfrac{b}{2}\omega, \quad v_R = v + \tfrac{b}{2}\omega
```
Velocities are peak-normalized to stay within $V_{max}$.

## Inputs and Expected Output
- **Inputs**: `get_pose()`, `get_target()`, `lidar()`, wheel base, max velocity.
- **Output**: `set_wheel_velocity(left, right)` each step.

## Score obtained with this approach

95.755 / 100
