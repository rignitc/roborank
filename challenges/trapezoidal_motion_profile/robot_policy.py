import math

class RobotPolicy:
    def step(self, robot):
        s = robot.get_state()
        t = robot.get_target()
        v_max, a_max = robot.limits()
        dt = robot.dt
        
        dx = t.position_m - s.position_m
        v = s.velocity_mps
        d = abs(dx)
        sign = 1 if dx > 0 else -1
        
        if d < 0.002 and abs(v) < 0.02:
            robot.set_acceleration(max(-a_max, min(a_max, -v / dt)))
            return
        
        max_safe = (-a_max*dt + math.sqrt((a_max*dt)**2 + 8*a_max*(d - 0.0005))) / 2
        
        v_brake = abs(v) - a_max*dt
        stop_dist = abs(v)*dt + v_brake*dt*(v_brake/(a_max*dt) + 1)/2
        
        if d <= stop_dist:
            v_des = max(0, abs(v) - a_max*dt)
        else:
            v_des = min(v_max, abs(v) + a_max*dt)
            if v_des > max_safe:
                v_des = max_safe
        
        robot.set_acceleration(max(-a_max, min(a_max, (sign*v_des - v)/dt)))