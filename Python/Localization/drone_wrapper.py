'''
Title: drone_slam.py
Author: Conor Green
Description: Library wrapper for ps_drone. Simplifies all commands
Usage: Run through another program
Version:
1.0 - June 13 2019 - Structure created
'''

#Imports
import time
import ps_drone

import numpy as np

def main(desired_data , **kwargs):
    options = {'req_take_off' : True}
    options.update(kwargs)

    drone = initialize_drone()

    #Determine packets to recieve
    drone.useDemoMode(False)
    drone.getNDpackage(desired_data)

    while req_take_off:
        if drone.State == 1:
            break
        time.sleep(.5)

    try:
        while True:
            gatherData()

    except KeyboardInterrupt:

        pass


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
	pass
