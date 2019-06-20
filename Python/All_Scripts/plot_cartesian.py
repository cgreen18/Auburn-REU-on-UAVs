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


    print("------posdata-------")
    print(pos_data)

    plot_3D(pos_data , options['sleeptime'] )

    return


def parse_flight_data(flight_data ):

    velocity_data = []

    for dict in flight_data:
        vel_data_t_slice = dict['demo'][4]
        vel_data_t_slice = vel_data_t_slice

        velocity_data.append(vel_data_t_slice)

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

def plot_3D(positions , sleeptime):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    X = positions[0,:]
    Y = positions[1,:]
    Z = positions[2,:]

    ax.scatter(X,Y,Z,c='r',marker='o')

    plt.show(block = True)
    plt.draw()
    plt.pause(.001)

    time.sleep(sleeptime)

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
    #Test data
    t_1 = {'demo' : [0 , 0 , [10,10,10] , 0 , [0,0,0] ]}
    t_2 = {'demo':[0 , 0 , [20,20,20] , 0 , [100 , 50 , 0]]}
    t_3 = {'demo' :  [0 , 0 , [30,30,30] , 0 , [100 , 50 , 100]]}
    flight_data = [t_1, t_2 , t_3]

    main(flight_data , sleeptime = 3 , dt = 1)

    #pass
