# Inverse Kinematics 2 (3-DOF Spatial)

## Approach
This policy provides a solution to the spatial IK of a 3 DOF robot arm. Firstly, it determines the base rotation $q_0$ by projecting the target location onto a horizontal plane, then calculating the planar reach and the effective vertical position with respect to the base. Then the 2DOF planar IK is solved for the remaining elbow and shoulder angles by determining the appropriate joint angles from the remaining joint space (either elbow-up or elbow-down configuration while meeting joint constraints).

## Major Equations
Base angle and horizontal reach projection:
$$ q_0 = \text{atan2}(y, x) $$
$$ r = \sqrt{x^2 + y^2} \cdot \cos(q_0^{clamped} - q_0) $$

Planar IK using the Law of Cosines for the remaining vertical plane $(r, z - H_{base})$:
$$ \cos(q_2) = \frac{r^2 + (z - H_{base})^2 - L_1^2 - L_2^2}{2 L_1 L_2} $$
$$ q_2 = \text{atan2}(\pm \sqrt{1 - \cos^2(q_2)}, \cos(q_2)) $$
$$ q_1 = \text{atan2}(z - H_{base}, r) - \text{atan2}(L_2 \sin(q_2), L_1 + L_2 \cos(q_2)) $$

## Inputs and Expected Output
- **Inputs**: Target position $(x, y, z)$, link lengths (`L1`, `L2`), base height (`H_base`), and joint limits.
- **Output**: Submits an array of joint angles `[q0, q1, q2]` to the arm.

## Score obtained with this approach

93.48 / 100