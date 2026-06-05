import math
import numpy as np

class RobotPolicy:
    def __init__(self):
        self.kp_pos = 2.5
        self.kp_att = 8.0
        self.kp_yaw = 4.5
        self.g = 9.81

    def step(self, robot) -> None:
        pose = robot.get_pose()
        gate = robot.get_next_gate()
        
        err = np.array([gate.x - pose.x, gate.y - pose.y, gate.z - pose.z])
        vel = np.array([pose.vx, pose.vy, pose.vz])
        dist = np.linalg.norm(err)
        
        if dist < 2.0:
            yaw_des = gate.yaw
        else:
            yaw_des = math.atan2(err[1], err[0])

        if dist > 0.5:
            ff = err / dist * np.array([2.2, 2.2, 1.7])
        else:
            ff = np.zeros(3)

        acc_des = self.kp_pos * err + self.kp_pos * (ff - vel)
        acc_des[2] += self.g

        cy, sy = math.cos(pose.yaw), math.sin(pose.yaw)
        ax_body = acc_des[0] * cy + acc_des[1] * sy
        ay_body = -acc_des[0] * sy + acc_des[1] * cy

        pitch_des = ax_body / self.g
        roll_des = -ay_body / self.g
        max_tilt = getattr(robot, 'max_tilt_rad', math.radians(45.0))
        pitch_des = max(-max_tilt, min(max_tilt, pitch_des))
        roll_des = max(-max_tilt, min(max_tilt, roll_des))

        power = (acc_des[2] / self.g) * robot.hover_power
        power = max(0.3, min(robot.max_power, power))

        roll_rate = self.kp_att * (roll_des - pose.roll)
        pitch_rate = self.kp_att * (pitch_des - pose.pitch)
        
        yaw_error = yaw_des - pose.yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        yaw_rate = self.kp_yaw * yaw_error
        
        max_rate = robot.max_body_rate_radps
        roll_rate = max(-max_rate, min(max_rate, roll_rate))
        pitch_rate = max(-max_rate, min(max_rate, pitch_rate))
        yaw_rate = max(-max_rate, min(max_rate, yaw_rate))
        
        robot.set_body_rate_and_power(roll_rate, pitch_rate, yaw_rate, power)