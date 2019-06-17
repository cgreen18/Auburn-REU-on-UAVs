'''
Title: drone_slam.py
Author: Conor Green
Description: Library wrapper for ps_drone. Simplifies all commands
Usage: Run through another program
Version:
1.0 - June 13 2019 - Structure created
1.1 - June 14 2019 - Gathering data fleshed out more. Organized return data well
1.2 - June 14 2019 - Changed data type of data dictionaries to lists from np arrays because it's hard to convert from the list of NavData
'''

#Imports
import time
import ps_drone

import numpy as np

'''
Initializes drone and deals with optional parameters. Gathers data and returns it
Return: flight_data - list of dictionaries of numpy arrays
        and the delta_t
'''
def main(drone , desired_data , **kwargs):
    print("within drone_slam")
    options = {'req_take_off' : False , 'demo' : True , 'time_lim' : 15}
    options.update(kwargs)

    #Determines packet rate
    #True = 15packets/s
    #False = 200pk/s
    drone.useDemoMode(options['demo'])

    if options['demo']:
        delta_t = 1/ 15
    else:
        delta_t = 1/200

    #Determine packets to recieve
    drone.getNDpackage(desired_data)

    #Wait for takeoff
    while options['req_take_off']:
        print("stuck here?")
        if drone.State == 1:
            options['req_take_off'] = False
            break
        time.sleep(.5)

    print("about to gather_flight)data")
    flight_data = gather_data_set_time(drone , options['time_lim'])
    print("Have gathered data")


    return (flight_data , delta_t)

'''
Gathers data as specified by the arguments in main for a specified amount of time
Returns: List of time slices of data (data is dictionary of lists ----not numpy arrays anymore)
'''
def gather_data_set_time(drone , time_lim):
    t_end = time.time() + time_lim

    print(t_end)

    flight_data = []
    last_NDC = drone.NavDataCount -1

    while time.time() < t_end:
        print(time.time())

        while drone.NavDataCount == last_NDC:
            print(drone.NavDataCount)
            #if drone.getKey():
            #	end = True
            time.sleep(.00045)

        last_NDC = drone.NavDataCount

        _data_slice = get_nav_frame(drone)
        flight_data.append(_data_slice)

    return flight_data


'''
Queries drone for navdata after waiting for new data and packages them nicely.
Has slim parameter to only take values deemed important
Return: Dictionary of numpy arrays and last navdatacount
'''

#TODO: Finish list of VISION later

def get_nav_frame(drone , *kwargs):

    print("Within navframe")
    print(drone.NavDataCount)
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
                'watchdog' : [] , 'trims' : range(0,4) , 'pwm' : range(0,12) , \
                'state' : []
                }
    ################################
    #TODO: Finish list of VISION later




    last_NDC = drone.NavDataCount

    data = {}

    if options['slim']:
        for _d_param in drone.NavData:

            data[_d_param] = []

            #for important element, copy that
            for _important_elem in important[_d_param]:
                data[_d_param].append(drone.NavData[_d_param][_important_elem])
            #data[_d_param] = np.array(drone.NavData[_d_param][_important_elems])

    else:
        for _d_param in drone.NavData:
            data[_d_param] = drone.NavData[_d_param]
            #data[_d_param] = np.array(drone.NavData[_d_param])

    print("\n++++++++++++++++++++++++++++++++++++")
    print(data)
    print(last_NDC)
    print("------------------------------------\n")

    return data


if __name__ == '__main__':
	pass
