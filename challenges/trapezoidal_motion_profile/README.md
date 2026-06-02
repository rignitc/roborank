# Trapezoidal Motion Profile

## Approach
This policy creates a 1D trapezoidal velocity profile for moving a robot to a desired position. The policy repeatedly calculates how far the robot needs to stop given its current velocity and maximum deceleration. If the robot is closer than the stopping distance, it will decelerate. Otherwise, it will accelerate toward the position given speed and acceleration constraints.

## Major Equations
The safe stopping distance is derived from kinematics:
$$ d_{stop} = v \cdot \Delta t + (v - a_{max}\Delta t) \cdot \Delta t \cdot \frac{\frac{v - a_{max}\Delta t}{a_{max}\Delta t} + 1}{2} $$

The target velocity $v_{des}$ is determined by checking if the remaining distance $d \le d_{stop}$:
$$ v_{des} = 
\begin{cases} 
\max(0, |v| - a_{max}\Delta t) & \text{if } d \le d_{stop} \\
\min(v_{max}, |v| + a_{max}\Delta t) & \text{otherwise}
\end{cases} $$

The commanded acceleration is then clamped:
$$ a_{cmd} = \text{clamp}\left( \frac{\text{sign}(\Delta x) \cdot v_{des} - v}{\Delta t}, -a_{max}, a_{max} \right) $$

## Inputs and Expected Output
- **Inputs**: Current state (position, velocity), target position, maximum velocity and acceleration limits, and time step (`dt`).
- **Output**: Submits a commanded acceleration to the robot via `set_acceleration()`.

## Score obtained with this approach

98.116 / 100