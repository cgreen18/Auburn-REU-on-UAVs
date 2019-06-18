'''
Title: plot_euler_angles.py
Author: Conor Green
Description: Script to plot euler angles given in main parameters in an animation
Usage: Called through main of Chief_Drone
Version:
1.0 - June 18 2019 - Created
1.1 - June 18 2019 - Tested and works
'''

import time

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from transforms3d import taitbryan

def main(flight_data , **kwargs):

    #time.sleep(10)

    options = {'sleeptime' : .5 , 'guess_reference' : False , 'real_time' : False , 'dt' :  0.005}
    options.update(kwargs)

    euler_angles = parse_flight_data(flight_data , options['guess_reference'])

    rotation_matricies = handle_angle_data(euler_angles)

    plot_3D(rotation_matricies , options['sleeptime'] , options['dt'])

    return

def parse_flight_data(flight_data , guesstimate):
    euler_angles = []

    if guesstimate:
        _dict = flight_data[0]
        offset = _dict['demo'][2]
    else:
        offset = [0,0,0]

    for dict in flight_data:
        angle_data_t_slice = dict['demo'][2]
        if not guesstimate:
            euler_angles.append(angle_data_t_slice)
        else:
            for i in range(0,3):
                angle_data_t_slice[i] = angle_data_t_slice[i] - offset[i]
            euler_angles.append(angle_data_t_slice)

    return euler_angles

def handle_angle_data(euler_angles ):
    #euler angles = [ [pitch_t0 , roll_t0 , yaw_t0] , [pitch_t1 , roll_t1 , yaw_t1] ...] in degrees

    list_of_rot_mats = []

    for t_slice in euler_angles:

        #Negatives added upon testing to assure it looked right
        pitch = np.deg2rad(-t_slice[0])
        roll = np.deg2rad(-t_slice[1])
        yaw = np.deg2rad(t_slice[2])

        _rotmat = taitbryan.euler2mat(yaw,pitch,roll)

        list_of_rot_mats.append(_rotmat)

    return list_of_rot_mats

def plot_3D(rot_mats , sleeptime , delta_t):
    fig = plt.figure()
    time.sleep(2)
    ax = Axes3D(fig)

    basis = np.array([[1,0,0] , [0,1,0] , [0,0,1]])
    basis = np.transpose(basis)
    last_t = time.time()
    for i in range(0,20):
        for rot_mat_t_slice in rot_mats:
            plt.cla()
            ax.set_xlim([-1.5 , 1.5])
            ax.set_ylim([1.5 , -1.5])
            ax.set_zlim([-1.5 , 1.5])
            #ax.view_init(0,90)

            for i in range(0,3):
                ax.quiver(0,0,0,rot_mat_t_slice[0] , rot_mat_t_slice[1] , rot_mat_t_slice[2] , length = 1)


            plt.show(block=False)
            plt.draw()
            plt.pause(.001)
            while time.time() < last_t + delta_t:
                time.sleep(.0001)
            last_t = time.time()

    return

if __name__ == '__main__':
    pass
