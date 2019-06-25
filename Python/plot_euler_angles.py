'''
Title: plot_euler_angles.py
Author: Conor Green
Description: Script to plot euler angles over time. Separate from any drone scripts; it does not import any drone scripts. Twin of plot_cartesian script.
Usage: Call from Chief_Drone with flight_data parameter or call from command line and use test data.
Version:
1.0 - June 18 2019 - Created
1.1 - June 18 2019 - Tested and works
1.2 - June 20 2019 - Finished!
'''

import time

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from transforms3d import taitbryan

#Handles kwargs and runs three main working functions: parse_flight_data , handle_angle_data, and plot_3D
def main(flight_data , **kwargs):

    options = {'sleeptime' : .01 , 'guess_reference' : False , 'real_time' : False , 'dt' :  0.005 , 'num_repeats' : 3}
    options.update(kwargs)

    euler_angles = parse_flight_data(flight_data , options['guess_reference'])

    rotation_matricies = handle_angle_data(euler_angles)

    plot_3D(rotation_matricies , options['real_time'], options['sleeptime'] , options['dt'] , options['num_repeats'])

    return

#Parses flight_data into euler angle data. Adjusts for original orientation if guesstimate parameter is True
#return: euler angles
def parse_flight_data(flight_data , guesstimate):
    euler_angles = [[0,0,0]]

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

#Turns euler angle data into rotational matricies.
#return: positions
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

#Plots the orientation over time in dynamic figure. Repeats by num_repeats parameter. Script ends after that many repetitions.
def plot_3D(rot_mats , real_time , sleeptime , delta_t , num_repeats):
    fig = plt.figure()
    time.sleep(2)
    ax = Axes3D(fig)

    basis = np.array([[1,0,0] , [0,1,0] , [0,0,1]])
    basis = np.transpose(basis)
    last_t = time.time()

    for i in range(0,num_repeats):
        for rot_mat_t_slice in rot_mats:
            plt.cla()
            ax.set_xlim([-1.5 , 1.5])
            ax.set_ylim([1.5 , -1.5])
            ax.set_zlim([-1.5 , 1.5])
            #ax.view_init(0,90)

            for i in range(0,3):
                ax.quiver(0,0,0, rot_mat_t_slice[0] , rot_mat_t_slice[1] , rot_mat_t_slice[2] , length = 1)


            plt.show(block=False)
            plt.draw()
            plt.pause(.001)
            if real_time:
                while time.time() < last_t + delta_t:
                    time.sleep(.0001)
                last_t = time.time()
            else:
                time.sleep(sleeptime)

    return

if __name__ == '__main__':
    #Test data
    t_1 = {'demo' : [0 , 0 , [10,10,10] , 0 , [0,0,0] ]}
    t_2 = {'demo':[0 , 0 , [20,20,20] , 0 , [100 , 50 , 0]]}
    t_3 = {'demo' :  [0 , 0 , [30,30,30] , 90 , [100 , 50 , 100]]}
    t_4 = {'demo' :  [0 , 0 , [40,40,40] , 180 , [100 , 50 , 100]]}
    t_5 = {'demo' :  [0 , 0 , [50,50,50] , 270 , [100 , 50 , 100]]}
    t_6 = {'demo' :  [0 , 0 , [60,60,60] , 360 , [100 , 50 , 100]]}
    flight_data = [t_1, t_2 , t_3 , t_4 , t_5, t_6]

    main(flight_data , sleeptime = .2)

    #pass
