'''
Title: drone_flying.py
Author: Conor Green
'''

import ps_drone

def main():
    drone = initialize_drone()

    return

def initialize_drone():
    drone = ps_drone.Drone()
    drone.startup()
    drone.reset()
    
    #Wait for reset to complete
    while(drone.getBattery()[0] == -1): time.sleep(.01)

    return drone

if __name__ == '__main__':
    main()
