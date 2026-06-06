import math
import numpy as np
class RobotPolicy:
    def step(self, robot):
        pose = robot.get_pose()
        target = robot.get_target()
        scan = robot.lidar()
        dx = target.x - pose.x
        dy = target.y - pose.y
        dist = math.hypot(dx, dy)
        goal_angle = math.atan2(dy, dx)
        err = (goal_angle - pose.yaw + math.pi) % (2 * math.pi) - math.pi
        MAX_V = robot.max_wheel_velocity_mps
        WB = robot.wheel_base_m
        n = len(scan)
        obs_steer = 0.0
        obs_brake = 1.0
        if n > 0:
            angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
            SAFE = 0.42
            HARD = 0.30
            danger = np.clip((SAFE - scan) / (SAFE - HARD), 0, 1) * (scan < SAFE)
            cos_w = np.clip(np.cos(angles), 0, None)
            obs_steer = float(np.sum(np.sin(angles) * danger * cos_w)) * -2.2
            fwd = np.abs(angles) < math.radians(35)
            fwd_min = float(scan[fwd].min()) if fwd.any() else SAFE
            obs_brake = max(0.0, min(1.0, (fwd_min - HARD) / (SAFE - HARD)))
        if dist < 0.22:
            v = 0.0
            w = 3.8 * err
        else:
            align = max(0.0, math.cos(err)) ** 2
            v = MAX_V * min(1.0, dist / 0.5) * align * obs_brake
            w_goal = 4.5 * err
            ow = 1.0 - obs_brake
            w = w_goal * (1.0 - ow * 0.6) + obs_steer * ow
        w = max(-4.0, min(4.0, w))
        left = v - 0.5 * WB * w
        right = v + 0.5 * WB * w
        peak = max(abs(left), abs(right))
        if peak > MAX_V:
            left *= MAX_V / peak
            right *= MAX_V / peak
        robot.set_wheel_velocity(left, right)