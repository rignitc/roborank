class RobotPolicy:
    KP, KI, KD = 6.3, 4.3, 6.0
    ALPHA, INTEGRATION_BAND = 0.25, 0.09

    def __init__(self):
        self.i = self.prev = self.df = 0.0
        self.init = False

    def step(self, motor):
        e = motor.target_force() - motor.scale_force()
        
        if not self.init:
            self.prev = e
            self.init = True

        if abs(e) < self.INTEGRATION_BAND:
            self.i += e * motor.dt

        self.df += self.ALPHA * ((e - self.prev) - self.df)

        ff = (motor.target_force() * motor.shaft_length_m) / motor.kt_nm_per_amp
        fb = self.KP * e + self.KI * self.i + self.KD * self.df
        cmd = ff + fb

        if abs(cmd) > motor.max_current_a:
            self.i -= e * motor.dt
            cmd = max(-motor.max_current_a, min(motor.max_current_a, cmd))

        self.prev = e
        motor.set_current(cmd)

        