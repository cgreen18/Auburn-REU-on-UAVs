'''
Author: Conor Green
Description:
Usage:
Version:
1.0 - June 6 2019 - Creation
'''

import numpy as np
import math

def roll_mat(phi):

    R_x_phi = np.zeros( (3,3) )
    R_x_phi[0][0] = 1

    phi_rad = np.deg2rad(phi)

    R_x_phi[1][1] = math.cos(phi_rad)
    R_x_phi[2][2] = math.cos(phi_rad)
    R_x_phi[1][2] = -math.sin(phi_rad)
    R_x_phi[2][1] = math.sin(phi_rad)

    return R_x_phi

def roll(phi , euler_angle):

    return roll_mat(phi)*euler_angle

def rotate(rots , ):
