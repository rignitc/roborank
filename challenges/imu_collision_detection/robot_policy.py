import math

class RobotPolicy:
    def __init__(self):
        self.prev = None
        self.accel_var = 1.0
        self.gyro_var = 0.5
        self.latched = False
        self.severity = "none"
        
    def step(self, robot):
        s = robot.imu()
        
        if self.latched:
            robot.submit_collision_decision(True, self.severity)
            return
        
        if not self.prev:
            self.prev = s
            self.prev_accel = math.hypot(s.ax, s.ay, s.az)
            self.prev_gyro = math.hypot(s.gx, s.gy, s.gz)
            robot.submit_collision_decision(False, "none")
            return
        
        dt = max(s.t - self.prev.t, 0.001)
        
        accel = math.hypot(s.ax, s.ay, s.az)
        gyro = math.hypot(s.gx, s.gy, s.gz)
        
        accel_jerk = abs(accel - self.prev_accel) / dt
        gyro_jerk = abs(gyro - self.prev_gyro) / dt
        
        self.accel_var = 0.98 * self.accel_var + 0.02 * (accel_jerk ** 2)
        self.gyro_var = 0.98 * self.gyro_var + 0.02 * (gyro_jerk ** 2)
        
        accel_z = accel_jerk / max(0.5, self.accel_var ** 0.5)
        gyro_z = gyro_jerk / max(0.3, self.gyro_var ** 0.5)
        
        if accel_z * gyro_z > 12.0:
            self.latched = True
            self.severity = "hard" if (accel_jerk > 35.0 or gyro_jerk > 15.0) else "light"
            robot.submit_collision_decision(True, self.severity)
            return
        
        robot.submit_collision_decision(False, "none")
        
        self.prev = s
        self.prev_accel = accel
        self.prev_gyro = gyro