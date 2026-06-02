# Motor Torque Scale Control

## Approach
This policy is used to scale and govern the torque applied to the motor to achieve the desired force using both Feedforward and PID combined controller. The feedforward controller directly maps the expected current required while the PID component compensates for the transient errors. An integration deadband and anti-windup clamping is provided to compensate the overshoot when the current reaches current limit.

## Major Equations
The commanded current is a combination of Feedforward (FF) and Feedback (FB):
$$ FF = \frac{F_{target} \cdot L_{shaft}}{K_t} $$
$$ FB = K_P \cdot e + K_I \int e \, dt + K_D \frac{de}{dt} $$
$$ I_{cmd} = \max(-I_{max}, \min(I_{max}, FF + FB)) $$

Where the error $e$ is:
$$ e = F_{target} - F_{scale} $$

## Inputs and Expected Output
- **Inputs**: Target force, current measured scale force, shaft length, torque constant ($K_t$), maximum current limit, and loop time step (`dt`).
- **Output**: Submits a current command $I_{cmd}$ to the motor using `set_current()`.

## Score obtained with this approach

91.5 / 100