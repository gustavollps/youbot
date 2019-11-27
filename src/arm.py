import rospy
import tf
import time
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trac_ik_python.trac_ik_wrap import TRAC_IK
import math
from utils import *


class Arm:
    gripper_pub = None
    joints_sub = None
    joints_pub = None
    bx = by = bz = 0.001
    brx = bry = brz = 0.001
    kdl_kin = None

    ik_solver = None

    def __init__(self, gripper_pub, joints_pub, urdf):
        self.joints_pub = joints_pub
        self.gripper_pub = gripper_pub
        self.ik_solver = TRAC_IK("base_link",
                                 "tool",
                                 urdf,
                                 0.1,  # default seconds
                                 1e-5,  # default epsilon
                                 "Speed")
        # print("Number of joints:")
        # print(self.ik_solver.getNrOfJointsInChain())
        # print("Joint names:")
        # print(self.ik_solver.getJointNamesInChain(urdf))
        # print("Link names:")
        # print(self.ik_solver.getLinkNamesInChain())

    def gripperClose(self, bool):
        msg = Bool()
        msg.data = bool
        self.gripper_pub.publish(msg)

    def solve(self, pos, quat, trans, rotat, q0):
        rotation = euler_from_quaternion(quat)
        # print(trans, dist_3d(pos, trans), dist_3d(rotation, rotat))
        factor = 5
        if trans is not None and (abs(pos[0] - trans[0]) > factor * self.bx or
                                  abs(pos[1] - trans[1]) > factor * self.by or
                                  abs(pos[2] - trans[2]) > factor * self.bz or
                                  abs(rotation[0] - rotat[0]) > factor * self.brx or
                                  abs(rotation[1] - rotat[1]) > factor * self.bry or
                                  abs(rotation[2] - rotat[2]) > factor * self.brz):
            sol = self.ik_solver.CartToJnt(q0,
                                           pos[0], pos[1], pos[2],
                                           quat[0], quat[1], quat[2], quat[3],
                                           self.bx, self.by, self.bz,
                                           self.brx, self.bry, self.brz)
            msg_cmd = Float64MultiArray()
            msg_cmd.data = sol
            if len(sol) != 0:
                self.joints_pub.publish(msg_cmd)
            else:
                # print('fail')
                return True, None
            # print('pub', sol)
            return True, sol
        else:
            return False, None


def forceSolve(self, pos, quat, trans, rotat, q0):
    sol = self.ik_solver.CartToJnt(q0,
                                   pos[0], pos[1], pos[2],
                                   quat[0], quat[1], quat[2], quat[3],
                                   self.bx, self.by, self.bz,
                                   self.brx, self.bry, self.brz)
    msg_cmd = Float64MultiArray()
    msg_cmd.data = sol
    if len(sol) != 0:
        # pose = self.kdl_kin.forward(sol)
        # _, _, rpy_angles, translation_vector, _ = tf.transformations.decompose_matrix(pose)
        # print(dist_3d(pos,trans))
        self.joints_pub.publish(msg_cmd)
        return True
    else:
        return False
