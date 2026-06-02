import math


class RobotPolicy:

    def step(self, arm):
        target = arm.get_target()
        limits = arm.joint_limits()
        x, y, z = target.x, target.y, target.z
        L1, L2, H_base = arm.link_1_m, arm.link_2_m, arm.base_height_m

        q0 = math.atan2(y, x)
        q0_clamped = max(limits[0][0], min(limits[0][1], q0))

        horiz_dist = math.sqrt(x**2 + y**2)
        q0_delta = q0_clamped - q0
        r = horiz_dist * math.cos(q0_delta)  # effective reach after q0 clamp

        z_rel = z - H_base
        d_sq = r**2 + z_rel**2
        d = math.sqrt(d_sq)

        if d > L1 + L2:
            q1 = math.atan2(z_rel, r)
            q2 = 0.0
        elif d < abs(L1 - L2) + 1e-6:
            raise ValueError(f"Target too close to reach (d={d:.4f} m)")
        else:
            cos_q2 = (d_sq - L1**2 - L2**2) / (2 * L1 * L2)
            cos_q2 = max(-1.0, min(1.0, cos_q2))  # guard float rounding

            best = None
            for sign in (1, -1):  # try elbow-down then elbow-up
                sin_q2 = sign * math.sqrt(1.0 - cos_q2**2)
                q2 = math.atan2(sin_q2, cos_q2)
                q1 = math.atan2(z_rel, r) - math.atan2(L2 * sin_q2, L1 + L2 * cos_q2)

                in_limits = (
                    limits[1][0] <= q1 <= limits[1][1] and
                    limits[2][0] <= q2 <= limits[2][1]
                )
                if in_limits:
                    best = (q1, q2)
                    break

            q1, q2 = best

        arm.submit_joint_angles([q0_clamped, q1, q2])