#!/usr/bin/env python

import controller
import arm
from utils import *

import rospy
import tf
import time
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

# start node
rospy.init_node('youbot_ik')

# get arm description for kinematics
urdf = rospy.get_param('/youbot_description')

# odom msg and angle variables
odom = Odometry()
odom_angle = [0, 0, 0]

# initial (and current) arm joint angles
qinit = [0.] * 6

# position and orientation for the inverse kinematics
x, y, z = 0, 0, 0
quat = quaternion_from_euler(-math.pi / 2, 0, 0)

# current position and orientation variables from tool tip to base_link
trans = None
rotat = None

# ROS transform listener object
listener = tf.TransformListener()

# simulation time
time_sim = 0

start_time = 0
state = 0
task_time = 0


# callback for the joints angles (with transform lookup)
def jointsCallBack(msg):
    global qinit
    qinit = msg.data


def simTime(msg):
    global time_sim
    time_sim = msg.data


def sensorCallBack(msg):
    global dist
    dist = msg.data


def odomCallBack(msg):
    global odom
    global odom_angle
    odom = msg
    # getting the roll, pitch and yaw (from quaternion)for the robot base
    quaternion = [odom.pose.pose.orientation.x,
                  odom.pose.pose.orientation.y,
                  odom.pose.pose.orientation.z,
                  odom.pose.pose.orientation.w]
    odom_angle = euler_from_quaternion(quaternion)


robot_name = 'youbot'
joints_sub = rospy.Subscriber('/' + robot_name + '/joints', Float64MultiArray, jointsCallBack)
time_sub = rospy.Subscriber('/' + robot_name + '/simTime', Float32, simTime)
gripper_sub = rospy.Subscriber('/' + robot_name + '/gripper_sensor', Float32, sensorCallBack)
odom_sub = rospy.Subscriber('/' + robot_name + '/pose', Odometry, odomCallBack)

joints_pub = rospy.Publisher('/' + robot_name + '/joints_cmd', Float64MultiArray, queue_size=1)
vel_pub = rospy.Publisher('/' + robot_name + '/joints_vel', Float64MultiArray, queue_size=1)
gripper_pub = rospy.Publisher('/' + robot_name + '/gripper', Bool, queue_size=1)
cmdvel_pub = rospy.Publisher('/' + robot_name + '/cmd_vel', Twist, queue_size=1)

# time delay to register the subscribers
time.sleep(0.5)

# object that holds the base movement functions
robot_control = controller.Controller(cmdvel_pub)

# object that holds the arm movement functions
arm_control = arm.Arm(gripper_pub, joints_pub, urdf)


# states machine for the task
def timerCallBack(event):
    global state
    global x, y, z, quat
    global task_time
    global trans, rotat

    # getting position and orientation for the arm's tip current pose
    try:
        (trans, r) = listener.lookupTransform('/base_link', '/tool', rospy.Time(0))
        (trans_map, r_map) = listener.lookupTransform('/map', '/tool', rospy.Time(0))
        rotat = euler_from_quaternion(r)
        rotat_map = euler_from_quaternion(r_map)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    # solve a inverse kinematics problem the desired [x,y,z] and quat
    ik_published = arm_control.solve([x, y, z], quat, trans, rotat, qinit)
    # ik_published
    # print(ik_published)

    if state == 0:
        state = 1
        x = 0
        y = 0.6
        z = 0.135
        arm_control.gripperClose(False)
    elif state == 1:
        pt = [-0.8, -2]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=True)
        if dist_2d(odom.pose.pose.position, pt) < 0.3:
            state = 2
            robot_control.stop()
    elif state == 2:
        pt = [-0.8, 1.4]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.3:
            state = 3
            robot_control.stop()
    elif state == 3:
        pt = [-2.2, 1.4]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.01:
            state = 4
            robot_control.stop()
    elif state == 4:
        pt = [-2.2, 1.6]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.005:
            state = 5
            robot_control.stop()
    elif state == 5:
        arm_control.gripperClose(True)
        state = 6
        task_time = time_sim
    elif state == 6:
        if time_sim - task_time > 2:
            state = 7
            task_time = time_sim
    elif state == 7:
        x = 0
        y = 0.6
        z = 0.20
        if time_sim - task_time > 1:
            state = 8
    elif state == 8:
        pt = [-2, -2]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.1:
            state = 9
            robot_control.stop()
    elif state == 9:
        pt = [1, -2]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.01:
            robot_control.stop()
            state = 10
    elif state == 10:  # -------------------- positioning arm -----------------------------------------------
        pt = [1, -0.51]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.01:
            x = 0
            z = 0.35
            y = 0.4
            quat = quaternion_from_euler(-math.pi / 2, math.pi, 0)
            state = 11
            task_time = time_sim
    elif state == 11:
        if time_sim - task_time > 3:
            state = 12
    elif state == 12:  # -------------------- deliver 0 -----------------------------------------------------
        pt = [2, -0.51]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.01:
            robot_control.stop()
            z = 0.35
            y = 0.4
            state = 13
            task_time = time_sim
    elif state == 13:
        robot_control.stop()
        arm_control.gripperClose(False)
        if time_sim - task_time > 0.5:
            y = 0.35
            state = 14
            task_time = time_sim
    elif state == 14:
        if time_sim - task_time > 1:
            pt = [1, -0.51]
            robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
            if dist_2d(odom.pose.pose.position, pt) < 0.1:
                state += 1
    elif state == 15:
        pt = [1, -2]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.1:
            state += 1
    elif state == 16:
        pt = [-0.8, -2]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.1:
            state += 1
    elif state == 17:
        pt = [-0.8, 1.4]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.1:
            state += 1
    elif state == 18:
        pt = [-1.9, 1.4]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.01:
            state += 1
    elif state == 19:
        pt = [-1.9, 1.6]
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)
        if dist_2d(odom.pose.pose.position, pt) < 0.05:
            state += 1
    elif state == 20:
        robot_control.stop()


if __name__ == '__main__':
    start_time = time_sim

    timer = rospy.Timer(rospy.Duration(0.02), timerCallBack)

    rospy.spin()
