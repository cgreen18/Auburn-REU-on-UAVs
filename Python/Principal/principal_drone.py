'''
Title: principal_drone.py
Author: Conor Green
Description: Runs everything
Usage: Call as main to thread all the processes
Public Methods:
Version:
1.0 - ~June 14 2019 - Created
1.1 - June 17 2019 - Completely chagned to work with new class structure of drone
'''

import ps_drone
import time
#import multiprocessing
#import threading

import plot_pos_cartesian
import manual_flight

def main():



    return
'''
Initialize instance of Drone class and start/reset drone.
Returns: drone (object)
'''
def initialize_drone():
    drone = ps_drone.Drone()
    drone.startup()
    drone.reset()

    #Wait for reset to complete
    while(drone.getBattery()[0] == -1): time.sleep(.01)

    return drone

if __name__ == '__main__':
    main()
