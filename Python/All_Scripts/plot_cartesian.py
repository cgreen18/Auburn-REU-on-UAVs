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

    vel_data = parse_flight_data(flight_data , options['guess_reference'])

    return




    pos = np.zeros((3,1))
    #temp
    offset = [0,0,0]

    #print(flight_data)

    for _dict in velocity_data:

        vel_data_t_slice = _dict['demo'][4]

        delta_pos = calc_delta_pos(vel_data_t_slice , delta_t)
        indx_last_t_slice = pos.shape[1] -1

        new_pos = pos[:,indx_last_t_slice].reshape(3,1) + delta_pos

        pos = np.append( pos , new_pos , axis=1 )


    #now pos.shape is (3 , num_t_slices) of type np.array
    plot_3D(pos)

    return

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

    time.sleep(1)

    return


'''
Converts velocity list to numpy arr and does simple math
Return: change in XYZ positions
'''
def calc_delta_pos(vels , delta_t):
    vels_np = np.array(vels).reshape((3,1))
    return vels_np*delta_t

if __name__ == '__main__':
    pass
