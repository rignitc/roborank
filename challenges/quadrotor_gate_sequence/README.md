# Quadrotor Gate Sequence

## Approach
This policy regulates the quadrotor using a series of gates via a cascaded position and attitude controller. It computes the position error to the next gate and outputs the commanded acceleration, including a velocity feedforward when far from the commanded location. Desired horizontal acceleration is rotated into the quadrotor body frame to calculate commanded roll and pitch angle. The desired thrust (power) is calculated using gravity compensation. Finally, the proportional attitude controller outputs commands in body rates based on roll, pitch and yaw error and these are saturated to limits before being sent to the quadrotor.

## Major Equations
The position error vector $\mathbf{e}_{pos}$ is:
```math
\mathbf{e}_{pos} = \mathbf{p}_{gate} - \mathbf{p}
```

The desired acceleration $\mathbf{a}_{des}$ incorporates a feedforward velocity command $\mathbf{v}_{ff}$ to track the trajectory:
```math
\mathbf{a}_{des} = K_{p,pos} \mathbf{e}_{pos} + K_{p,pos} (\mathbf{v}_{ff} - \mathbf{v})
```
where:
```math
\mathbf{v}_{ff} = \begin{cases} 
\frac{\mathbf{e}_{pos}}{\|\mathbf{e}_{pos}\|} \cdot [2.2, 2.2, 1.7]^T & \text{if } \|\mathbf{e}_{pos}\| > 0.5 \text{ m} \\
\mathbf{0} & \text{otherwise}
\end{cases}
```
Gravity compensation is added directly to the vertical desired acceleration:
```math
a_{des, z} = a_{des, z} + g
```

The desired horizontal acceleration is projected into the body frame using the current yaw angle $\psi$:
```math
a_{body, x} = a_{des, x} \cos(\psi) + a_{des, y} \sin(\psi)
```
```math
a_{body, y} = -a_{des, x} \sin(\psi) + a_{des, y} \cos(\psi)
```

Desired pitch ($\theta_{des}$) and roll ($\phi_{des}$) angles are then derived and clamped to the tilt limit $\theta_{max}$:
```math
\theta_{des} = \text{clamp}\left(\frac{a_{body, x}}{g}, -\theta_{max}, \theta_{max}\right)
```
```math
\phi_{des} = \text{clamp}\left(-\frac{a_{body, y}}{g}, -\theta_{max}, \theta_{max}\right)
```

The power command $P$ is determined relative to the hover power $P_{hover}$:
```math
P = \text{clamp}\left(\frac{a_{des, z}}{g} \cdot P_{hover}, 0.3, P_{max}\right)
```

Proportional control translates orientation errors into roll rate ($p$), pitch rate ($q$), and yaw rate ($r$) commands, which are subsequently clamped to the maximum body rate limit $\omega_{max}$:
```math
p_{cmd} = \text{clamp}(K_{p,att} (\phi_{des} - \phi), -\omega_{max}, \omega_{max})
```
```math
q_{cmd} = \text{clamp}(K_{p,att} (\theta_{des} - \theta), -\omega_{max}, \omega_{max})
```
```math
r_{cmd} = \text{clamp}(K_{p,yaw} \cdot \text{wrap}(\psi_{des} - \psi), -\omega_{max}, \omega_{max})
```
where the target yaw $\psi_{des}$ aligns with the gate orientation when nearby, or points toward the gate when distant:
```math
\psi_{des} = \begin{cases}
\psi_{gate} & \text{if } \|\mathbf{e}_{pos}\| < 2.0 \text{ m} \\
\text{atan2}(e_{pos, y}, e_{pos, x}) & \text{otherwise}
\end{cases}
```

## Inputs and Expected Output
- **Inputs**: Robot pose (position, linear velocity, orientation angles roll/pitch/yaw), next gate pose (position and orientation yaw), and robot-specific parameters (tilt limit, hover power, max power, body rate limit).
- **Output**: Submits body rates (roll rate, pitch rate, yaw rate) and power command via `set_body_rate_and_power()`.

## Score obtained with this approach

97.341 / 100
