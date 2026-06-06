import math
from collections import deque


class RobotPolicy:
    def __init__(self):
        self.buf = deque(maxlen=20)
        self.baseline_samples = []
        self.baseline_mean = None
        self.baseline_var = None
        self.calibrated = False
        self.collision_detected = False

    def step(self, robot):
        s = robot.imu()
        mag = math.sqrt(s.ax**2 + s.ay**2 + s.az**2)
        gmag = math.sqrt(s.gx**2 + s.gy**2 + s.gz**2)
        self.buf.append((mag, gmag))

        if not self.collision_detected:
            if not self.calibrated:
                self.baseline_samples.append(mag)
                if len(self.baseline_samples) >= 15:
                    m = sum(self.baseline_samples) / len(self.baseline_samples)
                    v = sum((x - m)**2 for x in self.baseline_samples) / len(self.baseline_samples)
                    self.baseline_mean = m
                    self.baseline_var = max(v, 0.01)
                    self.calibrated = True
                robot.submit_collision_decision(False, "none")
                return

            std = math.sqrt(self.baseline_var)
            z = (mag - self.baseline_mean) / std
            gyro_spike = gmag > 2.5

            if (z > 6.0 and gyro_spike) or z > 8.0:
                severity = "hard" if z > 10.0 else "light"
                self.collision_detected = True
                robot.submit_collision_decision(True, severity)
                return

        robot.submit_collision_decision(self.collision_detected, "none" if not self.collision_detected else "light")