'''
Title: plot_euler_angles.py
Author: Conor Green
Description: Script to plot euler angles given in main parameters in an animation
Usage: Called through main of Chief_Drone
Version:
1.0 - June 18 2019 - Created
'''

import time

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def main(euler_angles , delta_t):
    #euler angles = [ [pitch_t0 , roll_t0 , yaw_t0] , [pitch_t1 , roll_t1 , yaw_t1] ...] in degrees

    list_of_rot_mats = []

    for t_slice in euler_angles:
        pitch = np.deg2rad(t_slice[0])
        roll = np.deg2rad(t_slice[1])
        yaw = np.deg2rad(t_slice[2])

        _rotmat = taitbryan.euler2mat(yaw,pitch,roll)

        list_of_rot_mats.append(_rotmat)

    plot_3D(list_of_rot_mats)

    return



def plot_3D():
    fig = plt.figure()
    ax = Axes3D(fig)

    basis = np.array([[1,0,0] , [0,1,0] , [0,0,1]])
    basis = np.transpose(basis)

    for rot_mat_t_slice in rot_mats:
        plt.cla()
        ax.set_xlim([-1.5 , 1.5])
        ax.set_ylim([-1.5 , 1.5])
        ax.set_zlim([-1.5 , 1.5])
        #ax.view_init(0,90)

        for i in range(0,3):
            ax.quiver(0,0,0,rot_mat_t_slice[0] , rot_mat_t_slice[1] , rot_mat_t_slice[2] , length = 1)


        plt.show(block=False)
        plt.draw()
        plt.pause(.001)
        time.sleep(1)

    return

    if __name__ == '__main__':
        pass
