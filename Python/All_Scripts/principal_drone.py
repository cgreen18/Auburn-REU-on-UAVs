'''
Title: principal_drone.py
Author: Conor Green
'''

import ps_drone
import time
import multiprocessing

import plot_pos_cartesian
import manual_flight

def main():

    timeout = time.time() + 60*3

    drone = initialize_drone()

    navdat = multiprocessing.Process(target = plot_pos_cartesian.main , args = (drone,10))
    navdat.daemon = False

    flying = multiprocessing.Process(target = manual_flight.rough_flying , args = (drone,))
    flying.daemon = False

    navdat.start()
    flying.start()

    navdat.join()
    flying.join()

    if(time.time() > timeout):
        drone.stop()
        drone.land()
        return

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
