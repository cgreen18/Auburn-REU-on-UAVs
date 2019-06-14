'''
Title: plot_pos_cartesian.py
Author: Conor Green
Description: Plots the position over time as the integral of XYZ velocity
Usage: Call as main, no parameters
Version:
1.0 - June 14 2019 - Created structure
1.1 - June 14 2019 - Finished rought draft. Untested
'''

import time

import numpy as np
#from scipy.integrate import

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import drone_slam

def main():
    ( flight_data , delta_t ) = drone_slam.main(['demo'] , time_lim = 1)

    pos = np.zeros((3,1))
    #temp
    offset = [0,0,0]

    #print(flight_data)

    for dictionary in flight_data:

        vel_data_t_slice = dictionary['demo'][4]

        delta_pos = calc_delta_pos(vel_data_t_slice , delta_t)
        indx_last_t_slice = pos.shape[1] -1

        new_pos = pos[:,indx_last_t_slice].reshape(3,1) + delta_pos

        pos = np.append( pos , new_pos , axis=1 )


    #now pos.shape is (3 , num_t_slices) of type np.array
    plot_3D(pos)

    return

def plot_3D(position):

    X = position[0,:]
    Y = position[1,:]
    Z = position[2,:]

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.scatter(X,Y,Z,c='r',marker='o')

    plt.hold(True)

    plt.show(block = False)
    plt.draw()
    plt.pause(.001)

    time.sleep(10)

    return


'''
Converts velocity list to numpy arr and does simple math
Return: change in XYZ positions
'''
def calc_delta_pos(vels , delta_t):
    vels_np = np.array(vels).reshape((3,1))
    return vels_np*delta_t

if __name__ == '__main__':
    main()
