# Inverse Kinematics 1 (2-DOF Planar)

## Approach
This policy computes required joint angles (q 0 and q 1) to achieve desired location (x, y) for a planar 2-DOF robotic arm. Law of Cosines is applied to compute the elbow angle q1 and then, using geometry we can calculate base angle q0 while enforcing angle constraints.

## Major Equations
Using the Law of Cosines, the elbow angle $q_1$ is found:
$$ \cos(q_1) = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2} $$
$$ q_1 = \text{atan2}\left(\sqrt{1 - \cos^2(q_1)}, \cos(q_1)\right) $$

The base angle $q_0$ is then computed:
$$ q_0 = \text{atan2}(y, x) - \text{atan2}(L_2 \sin(q_1), L_1 + L_2 \cos(q_1)) $$

## Inputs and Expected Output
- **Inputs**: Target position $(x, y)$, link lengths (`L1`, `L2`), and joint limits.
- **Output**: Submits an array of joint angles `[q0, q1]` to the arm.

## Score obtained with this approach

93.37 / 100