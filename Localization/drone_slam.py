'''
Title: drone_slam.py
Author: Conor Green
Description: Library wrapper for ps_drone. Simplifies all commands
Usage: Run through another program
Version:
1.0 - June 13 2019 - Structure created
1.1 - June 14 2019 - Gathering data fleshed out more. Organized return data well
'''

#Imports
import time
import ps_drone

import numpy as np

'''
Initializes drone and deals with optional parameters. Gathers data and returns it
Return: flight_data - list of dictionaries of numpy arrays
'''
'''
'''
def main(desired_data , **kwargs):
    options = {'req_take_off' : False , 'demo' : True , 'time_lim' : 60}
    options.update(kwargs)

    drone = initialize_drone()

    #Determines packet rate
    #True = 15packets/s
    #False = 200pk/s
    drone.useDemoMode(options['demo'])

    #Determine packets to recieve
    drone.getNDpackage(desired_data)

    #Wait for takeoff
    while req_take_off:
        if drone.State == 1:
            req_take_off = False
            break
        time.sleep(.5)

    flight_data = gather_data_set_time(options['time_lim'])


    return flight_data

'''
Gathers data as specified by the arguments in main for a specified amount of time
Returns: List of time slices of data (data is dictionary of numpy arrays)
'''
def gather_data_set_time(time_lim):
    t_end = time.time() + time_lim

    flight_data = []
    _last_NDC = drone.NavDataCount

    while time.time() < t_end:
        ( _data_slice , _last_NDC ) = get_nav_frame(_last_NDC)
        flight_data.append(_data_slice)

    return flight_data


'''
Queries drone for navdata after waiting for new data and packages them nicely.
Has slim parameter to only take values deemed important
Return: Dictionary of numpy arrays and last navdatacount
'''

#TODO: Finish list of VISION later

def get_nav_frame(last_NDC , *kwargs):

    options = {'slim' : True}
    options.update(kwargs)

    #For slim, important indicies of the parameter are given
    #All of demo is important*   *maybe
    #none of zimmu_3000 is important because it is unknown
    important = {'demo': range(0,12) , 'time' : [0], 'wifi' : [0] , \
                'magneto': range(0,4) , 'altitude' : range(0,5) , 'pressure_raw': [0,1] , \
                'wind_speed' : [0 , 1] , 'kalman_pressure':[1] , 'zimmu_3000':[] , \
                'raw_measures' : range(0,4) , 'phys_measures' : [2 , 3] , 'references' : range(0,5) , \
                'rc_references' : range(0,5) , 'gyros_offsets' : [0] , 'euler_angles' : [0 ,1] , \
                'watchdog' : [] , 'trims' : range(0,4) , 'pwm' : range(0,12)

                }
    ################################
    #TODO: Finish list of VISION later


    while drone.NavDataCount == last_NDC:
        #if drone.getKey():
        #	end = True
        time.sleep(.002)

    last_NDC = drone.NavDataCount

    data = {}

    if slim:
        for _d_param in drone.NavData:
            #for important element, copy that
            _important_elems = important[_d_param]
            data[_d_param] = np.array(drone.NavData[_d_param][_important_elems])

    else:
        for _d_param in drone.NavData:
            data[_d_param] = np.array(drone.NavData[_d_param])



    return (data , last_NDC)

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
