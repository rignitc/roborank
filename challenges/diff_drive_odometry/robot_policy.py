from math import pi,cos,sin
class RobotPolicy:
    def __init__(self):
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.prev_left, self.prev_right = 0, 0

    def step(self, robot):
        L, R = robot.get_encoder_values()
        dl = (L-self.prev_left)  * (2*pi*robot.wheel_radius_m/robot.ticks_per_rev)
        dr = (R-self.prev_right) * (2*pi*robot.wheel_radius_m/robot.ticks_per_rev)
        self.prev_left, self.prev_right = L, R

        ds = (dl + dr) / 2.0
        dgy = robot.gyro() * robot.dt          # prefer gyro for heading

        self.yaw += dgy
        self.x   += ds * cos(self.yaw)
        self.y   += ds * sin(self.yaw)

        robot.submit_odometry(Pose2d(self.x, self.y, self.yaw))
