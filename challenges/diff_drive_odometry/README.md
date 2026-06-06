# Differential Drive Odometry

## Approach
Encoder-first odometry with continuous gyro bias estimation. Yaw is derived almost entirely from wheel encoder differentials, with the gyro used only during pure rotation or when both signals agree closely. When the robot is stationary (zero tick delta on both wheels), gyro samples accumulate into a rolling bias estimate that is subtracted on every step. A disagreement gate discards gyro yaw entirely when it deviates from encoder yaw by more than 0.3 rad/step, treating large divergence as noise rather than signal.

## Major Equations
Wheel displacements from cumulative tick deltas:
```math
\Delta d_{L,R} = \Delta N_{L,R} \cdot \frac{2\pi r}{N_{rev}}
```

Center displacement and encoder-derived yaw rate:
```math
\Delta s = \frac{\Delta d_L + \Delta d_R}{2}, \quad \Delta\theta_{enc} = \frac{\Delta d_R - \Delta d_L}{b}
```

Bias-corrected gyro yaw:
```math
\Delta\theta_{gyro} = (\omega - \hat{b}) \cdot \Delta t, \quad \hat{b} = \frac{1}{N}\sum_{stationary} \omega_i
```

Fused heading with disagreement-gated alpha:
```math
\Delta\theta = \alpha \cdot \Delta\theta_{gyro} + (1 - \alpha) \cdot \Delta\theta_{enc}, \quad \alpha = f(|\Delta\theta_{gyro} - \Delta\theta_{enc}|)
```

Midpoint integration:
```math
x_{k+1} = x_k + \Delta s\cos\!\left(\theta_k + \tfrac{\Delta\theta}{2}\right), \quad y_{k+1} = y_k + \Delta s\sin\!\left(\theta_k + \tfrac{\Delta\theta}{2}\right)
```

## Inputs and Expected Output
- **Inputs**: Cumulative encoder ticks (`L`, `R`), wheel geometry, gyro reading `ω`, timestep `dt`.
- **Output**: Submits estimated `Pose2d(x, y, yaw)` each step.

## Score obtained with this approach

97.975 / 100
