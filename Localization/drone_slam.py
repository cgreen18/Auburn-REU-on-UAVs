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
    options = {'req_take_off' : False , 'demo' : False}
    options.update(kwargs)

    drone = initialize_drone()

    #True = 15packets/s
    #False = 200pk/s
    drone.useDemoMode(options['demo'])
    #Determine packets to recieve
    drone.getNDpackage(desired_data)

    while req_take_off:
        if drone.State == 1:
            req_take_off = False
            break
        time.sleep(.5)

    try:
        while True:
            gather_data()

    except KeyboardInterrupt:

        pass


    return

'''
Gathers data as specified by the arguments in main() and packages them nicely dictionary of numpy arrays
Return: Diction
'''
def gather_data():

    return

def get_nav_frame(last_NDC , *kwargs):

    while drone.NavDataCount == last_NDC:
        #if drone.getKey():
        #	end = True
        time.sleep(.001)
    last_NDC = drone.NavDataCount

    if slim:

    else:



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
