'''
Title: plot_cartesian.py
Author: Conor Green
Description: Plots the flight path (position) through dead reckoning estimation of XYZ velocity. Separate from any drone scripts; it does not import any drone scripts. Twin of plot_euler_angles script.
Usage: Call from Chief_Drone with flight_data parameter or call from command line and use test data.
Version:
1.0 - June 14 2019 - Copy/pasted from plot_pos_cartesian (old script) and modified to adhere to new class structure of chief_drone
1.1 - June 20 2019 - Split up work into three functions
1.2 - June 20 2019 - Finished!
'''

import time

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

#Handles kwargs and runs three main working functions: parse_flight_data , handle_vel_data, and plot_3D
def main(flight_data , **kwargs):

    options = {'sleeptime' : .5 , 'guess_reference' : False , 'real_time' : False , 'dt' :  0.005}
    options.update(kwargs)

    vel_data = parse_flight_data(flight_data )

    pos_data = handle_vel_data(vel_data , options['dt'] , options['guess_reference'])

    plot_3D(pos_data )

    return

#Parses flight_data into velocity data
#return: velocities
def parse_flight_data(flight_data ):

    velocity_data = []

    for dict in flight_data:
        vel_data_t_slice = dict['demo'][4]
        vel_data_t_slice = vel_data_t_slice

        velocity_data.append(vel_data_t_slice)

    return velocity_data


#Turns velocity data into position data. Adjusts for original position if guesstimate option is True
#return: positions
def handle_vel_data(velocities , dt , guesstimate):

    if guesstimate:
        pos = calc_delta_pos(velocities[0] , dt)
    else:
        pos = np.zeros((3,1))

    for vel in velocities:

        delta_pos = calc_delta_pos(vel , dt)
        indx_last_t_slice = pos.shape[1] -1

        new_pos = pos[:,indx_last_t_slice].reshape(3,1) + delta_pos

        pos = np.append( pos , new_pos , axis=1 )

    return pos


# Plots XYZ positions in static 3D figure. Exit figure to end script.
def plot_3D(positions):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    X = positions[0,:]
    Y = positions[1,:]
    Z = positions[2,:]

    ax.scatter(X,Y,Z,c='r',marker='o')

    plt.draw()
    plt.show(block = True)
    plt.pause(.001)

    return


# Converts velocity list to numpy arr and does simple math. Uses dead reckoning logic
# Return: change in XYZ positions
def calc_delta_pos(vels , delta_t):
    vels_np = np.array(vels).reshape((3,1))
    return vels_np*delta_t

if __name__ == '__main__':
    #Test data
    t_1 = {'demo' : [0 , 0 , [10,10,10] , 0 , [0,0,0] ]}
    t_2 = {'demo':[0 , 0 , [20,20,20] , 0 , [100 , 50 , 0]]}
    t_3 = {'demo' :  [0 , 0 , [30,30,30] , 0 , [100 , 50 , 100]]}
    flight_data = [t_1, t_2 , t_3]

    main(flight_data , sleeptime = 3 , dt = 1)

    #pass
