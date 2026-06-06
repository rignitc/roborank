import math


class RobotPolicy:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_left = None
        self.prev_right = None
        self.gyro_bias = 0.0
        self.bias_samples = []

    def step(self, robot):
        left_ticks, right_ticks = robot.get_encoder_values()
        yaw_rate = robot.gyro()

        if self.prev_left is None:
            self.prev_left = left_ticks
            self.prev_right = right_ticks
            robot.submit_odometry(Pose2d(0.0, 0.0, 0.0))
            return

        meters_per_tick = (2 * math.pi * robot.wheel_radius_m) / robot.ticks_per_rev

        dl = (left_ticks - self.prev_left) * meters_per_tick
        dr = (right_ticks - self.prev_right) * meters_per_tick
        self.prev_left = left_ticks
        self.prev_right = right_ticks

        d_enc = (dl + dr) / 2.0
        dyaw_enc = (dr - dl) / robot.wheel_base_m

        if dl == 0.0 and dr == 0.0:
            self.bias_samples.append(yaw_rate)
            if len(self.bias_samples) > 100:
                self.bias_samples.pop(0)
            self.gyro_bias = sum(self.bias_samples) / len(self.bias_samples)

        corrected_yaw_rate = yaw_rate - self.gyro_bias
        dyaw_gyro = corrected_yaw_rate * robot.dt

        dyaw_diff = abs(dyaw_gyro - dyaw_enc)

        if dl == 0.0 and dr == 0.0:
            dyaw = dyaw_gyro
        elif abs(dyaw_enc) < 1e-6:
            dyaw = 0.05 * dyaw_gyro + 0.95 * dyaw_enc
        elif dyaw_diff > 0.3:
            dyaw = dyaw_enc
        else:
            w = min(1.0, dyaw_diff / 0.3)
            alpha = 0.9 * (1.0 - w) + 0.0 * w
            dyaw = alpha * dyaw_gyro + (1.0 - alpha) * dyaw_enc

        mid_yaw = self.yaw + dyaw / 2.0
        self.x += d_enc * math.cos(mid_yaw)
        self.y += d_enc * math.sin(mid_yaw)
        self.yaw += dyaw
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

        robot.submit_odometry(Pose2d(self.x, self.y, self.yaw))