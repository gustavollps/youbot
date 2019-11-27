import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def dist_2d(odom_p, pos):
    dx = odom_p.x - pos[0]
    dy = odom_p.y - pos[1]
    return math.sqrt(dx ** 2 + dy ** 2)


def dist_3d(p1, p2):
    if p1 is not None and p2 is not None:
        d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)
        return d
    else:
        return 9999999.0


def constrain(x, xlim):
    if x > xlim:
        x = xlim
    elif x < -xlim:
        x = -xlim

    return x


def fix_angle(angle, odom_angle):
    d_angle = angle - odom_angle
    if abs(d_angle) > math.pi:
        if odom_angle > 0:
            angle = angle + 2 * math.pi
        else:
            angle = angle - 2 * math.pi

    return angle


def get_angle(odom):
    quaternion = [odom.pose.pose.orientation.x,
                  odom.pose.pose.orientation.y,
                  odom.pose.pose.orientation.z,
                  odom.pose.pose.orientation.w]
    return euler_from_quaternion(quaternion)


def format_dec(num, places):
    factor = 10 ** places
    return math.floor(num * factor) / factor


def format_vec(vec, places):
    output = [0] * len(vec)
    for i, x in enumerate(vec):
        output[i] = format_vec(x, places)
    return output


def mult_vec(vec, factor):
    output = [0] * len(vec)
    for i, x in enumerate(vec):
        output[i] = x * factor
    return output
