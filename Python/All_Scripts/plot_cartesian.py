'''
Title: plot_cartesian.py
Author: Conor Green
Description: Plots the position over time as the integral of XYZ velocity. Separate from any drone scripts: it asks chief_drone for navdata as numpy arrays and simply plots them.
Usage: Call as main
Version:
1.0 - June 14 2019 - Copy/pasted from plot_pos_cartesian (old script) and modified to adhere to new class structure of chief_drone
'''

import time

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def main(flight_data , **kwargs):

    options = {'sleeptime' : .5 , 'guess_reference' : False , 'real_time' : False , 'dt' :  0.005}
    options.update(kwargs)

    vel_data = parse_flight_data(flight_data )

    pos_data = handle_vel_data(vel_data , options['dt'] , options['guess_reference'])

    plot_3D(pos_data , options['real_time'] , options['sleeptime'] , options['dt'])

    return


def parse_flight_data(flight_data ):

    velocity_data = []

    if guesstimate:
        _dict = flight_data[0]
        offset = _dict['demo'][4]
    else:
        offset = [0,0,0]

    for dict in flight_data:
        vel_data_t_slice = _dict['demo'][4]


        vel_data_t_slice.append(angle_data_t_slice)

    return velocity_data


def handle_vel_data(velocities , dt , guesstimate):

    #temp
    if guesstimate:
        pos = calc_delta_pos(velocities[0] , dt)
    else:
        pos = np.zeros((3,1))

    #print(flight_data)

    for vel in velocities:

        delta_pos = calc_delta_pos(vel , dt)
        indx_last_t_slice = pos.shape[1] -1

        new_pos = pos[:,indx_last_t_slice].reshape(3,1) + delta_pos

        pos = np.append( pos , new_pos , axis=1 )


    #now pos.shape is (3 , num_t_slices) of type np.array

    return pos

def plot_3D(positions):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    X = positions[0,:]
    Y = positions[1,:]
    Z = positions[2,:]

    ax.scatter(X,Y,Z,c='r',marker='o')

    plt.hold(True)

    plt.show(block = True)
    plt.draw()
    plt.pause(.001)

    time.sleep(5)

    return


'''
Converts velocity list to numpy arr and does simple math.
Uses dead reckoning logic
Return: change in XYZ positions
'''
def calc_delta_pos(vels , delta_t):
    vels_np = np.array(vels).reshape((3,1))
    return vels_np*delta_t

if __name__ == '__main__':
    pass
