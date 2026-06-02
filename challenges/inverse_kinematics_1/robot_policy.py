import math


class RobotPolicy:
    def step(self, arm):
        target = arm.get_target()
        limits = arm.joint_limits()

        x, y = target.x, target.y
        L1 = arm.link_1_m
        L2 = arm.link_2_m
        d_sq = x**2 + y**2
        cos_q1 = (d_sq - L1**2 - L2**2) / (2 * L1 * L2)
        cos_q1 = max(-1.0, min(1.0, cos_q1))
        sin_q1 = math.sqrt(1.0 - cos_q1**2)
        q1 = math.atan2(sin_q1, cos_q1)
        q0 = math.atan2(y, x) - math.atan2(L2 * sin_q1, L1 + L2 * cos_q1)

        def clamp(val, lo, hi):
            return max(lo, min(hi, val))

        q0 = clamp(q0, limits[0][0], limits[0][1])
        q1 = clamp(q1, limits[1][0], limits[1][1])

        arm.submit_joint_angles([q0, q1])
