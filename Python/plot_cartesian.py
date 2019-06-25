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

    options = {'sleeptime' : .5 , 'real_time' : False , 'dt' :  1/200}
    options.update(kwargs)

    (vel_data , alt_data) = parse_flight_data(flight_data )

    pos_vel_data = handle_vel_data(vel_data , options['dt'])

    pos_alt_data = handle_alt_data(alt_data)

    pos_data = average_z_height(pos_vel_data , pos_alt_data)

    plot_3D(pos_data )

    return

#Parses flight_data into velocity and altitude data
#return: velocities and altitudes
def parse_flight_data(flight_data ):

    velocity_data = []
    altitude_data = []

    for dict in flight_data:
        alt_data_t_slice = dict['demo'][3]

        altitude_data.append(alt_data_t_slice)

        vel_data_t_slice = dict['demo'][4]

        velocity_data.append(vel_data_t_slice)

    return (velocity_data , altitude_data)

#Adds extra zero position to match size of velocity and turns centimeters into milimeters (like velocity)
def handle_alt_data(alt_list):

    alt_data = np.array(alt_list)
    alt_data *= 10

    _temp_alt = np.zeros(1)

    alt_data = np.append( _temp_alt , alt_data)

    return alt_data

#Turns velocity data into position data. Adjusts for original position if guesstimate option is True.
#return: positions
def handle_vel_data(velocities , dt):

    pos = np.zeros((3,1))

    for vel in velocities:

        delta_pos = calc_delta_pos(vel , dt)
        indx_last_t_slice = pos.shape[1] -1

        new_pos = pos[:,indx_last_t_slice].reshape(3,1) + delta_pos

        pos = np.append( pos , new_pos , axis=1 )


    return pos

def average_z_height(vel_pos , alt):

    num_pts = len(alt)

    pos_data = vel_pos[0:2 , :]

    avg = np.mean(np.array([vel_pos[2,:] , alt]) , axis=0)
    avg = avg.reshape(1,-1)
    pos_data = np.append(pos_data , avg , axis=0)

    return pos_data


# Plots XYZ positions in static 3D figure. Exit figure to end script.
def plot_3D(positions):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    X = positions[0,:]
    Y = -positions[1,:]
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
    t_3 = {'demo' :  [0 , 0 , [30,30,30] , 9.0 , [100 , 50 , 100]]}
    t_4 = {'demo' :  [0 , 0 , [40,40,40] , 18.0 , [100 , 50 , 100]]}
    t_5 = {'demo' :  [0 , 0 , [50,50,50] , 27.0 , [100 , 50 , 100]]}
    t_6 = {'demo' :  [0 , 0 , [60,60,60] , 36.0 , [100 , 50 , 100]]}
    flight_data = [t_1, t_2 , t_3 , t_4 , t_5, t_6]

    main(flight_data , sleeptime = 3 , dt = 1)

    #pass
