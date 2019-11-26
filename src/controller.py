import rospy
from geometry_msgs.msg import Twist
from utils import *


class Controller:
    kp = 0.4
    ki = 0.01
    kd = 0.5
    I = 0
    error = 0
    old_error = 0
    cmdvel_pub = None
    vellim = 0.8

    move_kp = 1
    move_kp_fast = 2

    def __init__(self, cmdvel_pub):
        self.cmdvel_pub = cmdvel_pub

    def setAngle(self, setpoint_angle, odom_angle):
        self.I = constrain(self.I, 0.5)
        setpoint_angle = fix_angle(setpoint_angle, odom_angle[2])
        self.error = (setpoint_angle - odom_angle[2])

        threshold = 30 * math.pi / 180
        if abs(self.error) < threshold:
            P = self.kp * self.error
            I = self.I + self.error * self.ki
            D = (self.error - self.old_error) * self.kd

            PID = P + I + D
            self.old_error = self.error
            # print(PID, error, angle, odom_angle[2])
            return PID
        else:
            controller_val = 0.5
            if self.error < 0:
                return -controller_val
            elif self.error > 0:
                return controller_val

    def moveCmd(self, x, y, rot):
        msg = Twist()
        x = constrain(x, self.vellim)
        y = constrain(y, self.vellim)
        msg.linear.x = y
        msg.linear.y = x
        msg.angular.z = rot
        self.cmdvel_pub.publish(msg)

    def moveLocal(self, x, y, odom):
        dx = x - odom.pose.pose.position.x
        dy = y - odom.pose.pose.position.y
        min_vel = 0.000
        if 0 < dx < min_vel:
            dx = min_vel
        elif -min_vel < dx < 0:
            dx = -min_vel
        if 0 < dy < min_vel:
            dy = min_vel
        elif -min_vel < dy < 0:
            dy = -min_vel

        # move_cmd(0, 0, rot)
        self.moveCmd(dx, dy, 0)

    def moveGlobal(self, point, ang, odom, fast):
        x = point[0]
        y = point[1]
        if fast:
            kp_m = self.move_kp_fast
        else:
            kp_m = self.move_kp

        dx = (x - odom.pose.pose.position.x) * kp_m
        dy = (y - odom.pose.pose.position.y) * kp_m

        odom_angle = get_angle(odom)
        yaw = odom_angle[2]
        vel_ang = math.atan2(dy, dx)
        vel_mod = math.sqrt(dx ** 2 + dy ** 2)
        dang = vel_ang - yaw
        sy = math.sin(dang)
        cy = math.cos(dang)
        vx = vel_mod * cy
        vy = vel_mod * sy
        rot = -self.setAngle(ang, odom_angle)
        vx = constrain(vx, 1)
        vy = constrain(vy, 1)
        ang = fix_angle(ang, yaw)
        if not fast:
            if abs(ang - yaw) > 5 * math.pi / 180:
                vx = vy = 0
            vx = constrain(vx, 1)
            vy = constrain(vy, 1)
            self.moveCmd(vx, vy, rot)
        else:
            vx = constrain(vx, 1.5)
            vy = constrain(vy, 1.5)
            self.moveCmd(vx, vy, rot)

    def stop(self):
        self.I = 0
        self.moveCmd(0, 0, 0)
