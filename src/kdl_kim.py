from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics as KDL
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class forwardKin:
    robot_urdf = None
    kdl_kin = None

    def __init__(self, urdf):
        self.robot_urdf = URDF.from_xml_string(urdf)
        self.kdl_kin = KDL(self.robot_urdf, "base_link", "tool")

    def fkin(self, angles):
        angles = [0, 0, 0, 0, 0]
        pose = self.kdl_kin.forward(angles)
        _, _, rpy_angles, translation_vector, _ = \
            tf.transformations.decompose_matrix(pose)
        print("rpy ---- ", rpy_angles, translation_vector)
        return translation_vector, rpy_angles
