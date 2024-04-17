import math
import quaternionic 
import numpy as np
from numpy import deg2rad as rad
from numpy import rad2deg as deg
from quaternionic import converters
from dual_quaternions import DualQuaternion

def parse_pose(file_name = None):
    if not file_name:
        return 
    poses = []
    with open(file_name, 'r') as file:
        for line in file:
            int_pose = [float(val) for val in line.split(',')]
            poses.append(int_pose)
            # print(poses)
            # continue
    return poses

def to_quaternion(roll, pitch, yaw):
    # Abbreviations for the various angular functions
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]

def to_euler_angles(q):
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1] ** 2 + q[2] ** 2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q[0] * q[2] - q[1] * q[3])
    pitch = math.asin(max(-1, min(1, sinp)))  # Clamped to handle numerical issues

    siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
    cosy_cosp = 1 - 2 * (q[2] ** 2 + q[3] ** 2)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]


# def run_motion():

#     parse_pose()

if __name__ == "__main__":
    parse_pose("test.txt")