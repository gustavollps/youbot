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
qinit = [0.] * 5

# position and orientation for the inverse kinematics
x, y, z = 0, 0.6, 0.135
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
dist = 0


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

    position = odom.pose.pose.position

    # getting position and orientation for the arm's tip current pose
    try:
        # get the current transform between the base_link and the tip of the tool
        (trans, r) = listener.lookupTransform('/base_link', '/tool', rospy.Time(0))
        # convert the quaternion to a readable roll, pitch and yaw angles
        rotat = euler_from_quaternion(r)

        # get the current transform between the map origin and the tip of the tool
        (trans_map, r_map) = listener.lookupTransform('/map', '/tool', rospy.Time(0))
        # convert the quaternion to a readable roll, pitch and yaw angles
        rotat_map = euler_from_quaternion(r_map)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    if state == 0:
        # variables to control the tip pose
        x = -0.35
        y = 0.16
        z = 0.13
        # row, pith and yaw angles
        rpy = (-math.pi / 2, 0, math.pi / 2)
        # conversion of row, pitch and yaw angles to a quaternion variable (necessity for the ik solver)
        quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])

        # point with desired position to go (x,y)
        pt = [-2.2, 1.4]

        # calculates the distance between the 'position' (that is odom.pose.pose.position)
        # and pt (a list with 2 elements)
        d = dist_2d(position, pt)

        # Controls the linear and angular velocities of the robot to reach
        # the point (point=VALUE) with the desired angle (ang=VALUE)
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)

        if d < 0.1:  # state changing trigger (if the robot is less than 0.1m from its objective
            state = 1

    elif state == 1:
        # variables to control the tip pose
        x = 0.0
        y = 0.6
        z = 0.25
        # row, pith and yaw angles
        rpy = (-math.pi / 2, 0, 0)
        # conversion of row, pitch and yaw angles to a quaternion variable (necessity for the ik solver)
        quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])

        # point with desired position to go (x,y)
        pt = [-2.2, 1.4]

        # calculates the distance between the 'position' (that is odom.pose.pose.position)
        # and pt (a list with 2 elements)
        d = dist_2d(position, pt)

        # Controls the linear and angular velocities of the robot to reach
        # the point (point=VALUE) with the desired angle (ang=VALUE)
        robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)

        if d < 0.1:  # state changing trigger (if the robot is less than 0.1m from its objective
            state = 1


# inverse kinematics timer (runs at lower frequency to lighten the computational load
def ik_timerCallBack(event):
    # This method states if it is necessary to move the arm to reach the desired position and orientation. If so, it
    # calculates the inverse kinematics, and then publishes it on the specified topic.
    # Returns true if there was a ik calculation try, and the solution if it was published correctly
    ik_calc, solution = arm_control.solve([x, y, z], quat, trans, rotat, qinit)
    if ik_calc:
        if solution is not None:
            print("Published solution:", solution)
        else:
            print("Solution failed")
    else:
        print('No IK needed, tip already at the desired pose')

    # NOTE -----------------------------------------------------------------------------------------------------------
    # the kinematics of may not allow some pose with the exactly set position/orientation
    # changing error limits for the inverse kinematics result's will allow more poses
    # bx, by, and bz are the x,y,z limits for the desired position
    arm_control.bx = 0.001
    arm_control.by = 0.001
    arm_control.bz = 0.001
    # brz modifies the limit for YAW (relative to the desired frame)
    arm_control.brz = 0.05
    # bry modifies the limit for PITCH (relative to the desired frame)
    arm_control.bry = 0.05
    # brx modifies the limit for ROLL (relative to the desired frame)
    arm_control.brx = 0.05

    # ----------------------------------------------------------------------------------------------------------------
    # force solving and publishing (if a successful calculation) the inverse kinematics results
    # Return true if the output angles were correctly calculated
    force = None
    if force is not None:
        ik_success = arm_control.forceSolve([x, y, z], quat, trans, rotat, qinit)


if __name__ == '__main__':
    start_time = time_sim

    # timer for the states machine and control loop for position and orientation (with robot_control.moveGlobal)
    timer = rospy.Timer(rospy.Duration(0.02), timerCallBack)

    # timer for the inverse kinematics (lower frequency due to the heavy load for the calculations)
    ik_timer = rospy.Timer(rospy.Duration(0.3), ik_timerCallBack)

    rospy.spin()
