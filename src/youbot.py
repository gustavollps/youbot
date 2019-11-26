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

    pose = odom.pose.pose.position

    x = 0.0
    y = 0.6
    z = 0.135

    # get the quaternion from a roll, pitch and yaw angles
    quaternion_from_euler(-math.pi / 2, 0, 0)

    # point with desired position to go
    pt = [-2.2, 1.4]

    # calculates the distance between the 'pose' (that is odom.pose.pose.position) and pt variables
    d = dist_2d(pose, pt)

    # Controls the linear and angular velocities of the robot to reach
    # the point (point=VALUE) with the desired angle (ang=VALUE)
    robot_control.moveGlobal(point=pt, ang=0, odom=odom, fast=False)

    # This method states if it is necessary to move the arm to reach the desired position and orientation. If so, it
    # calculates the inverse kinematics, and then publishes it on the specified topic.
    # Returns true if the data were calculated and published correctly
    ik_published = arm_control.solve([x, y, z], quat, trans, rotat, qinit)

    # NOTE -----------------------------------------------------------------------------------------------------------
    # the kinematics of may not allow some pose with the exactly orientation
    # changing error limits for the inverse kinematics result's orientation will allow more positions
    # brz modifies the limit for YAW
    arm_control.brz = 9999
    # bry modifies the limit for PITCH
    arm_control.bry = 0.001
    # brx modifies the limit for ROLL
    arm_control.brx = 0.001

    # force solving and publishing (if a successful calculation) the inverse kinematics results
    # Return true if the output angles were correctly calculated
    ik_published = arm_control.forceSolve([x, y, z], quat, trans, rotat, qinit)


if __name__ == '__main__':
    start_time = time_sim

    timer = rospy.Timer(rospy.Duration(0.02), timerCallBack)

    rospy.spin()
