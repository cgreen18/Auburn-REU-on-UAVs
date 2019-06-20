'''
Title: Chief_Drone.py
Author: Conor Green and Brenden Stevens
Description: Class to maintain the same drone object (from ps_drone). Holds methods to get navdata and fly
Usage: Call from principal_drone to multiprocess navdata and flying
Public Methods:
    get_navdata - Queries the drone for navdata for a set period of time. See method for kwargs arguments
Version:
1.0 - June 17 2019 - Initial creation. Copy/pasted methods from previous scripts
1.1 - June 18 2019 - Changed main
1.2 - June 18 2019 - Commented
1.3 - June 20 2019 - Added fly and track that utilizes threading (custom thread class - Drone_Thread)
'''

#Standard lib imports
import time
import sys

#Extended library
import numpy as np

#Philip's backbone import
import ps_drone_vp3

#Imports from this project
import Drone_Thread
import plot_cartesian
import plot_euler_angles


### TODOs: see __init__
###
### fly_and_track
###
### time stamp
### Global time stamp?

class Chief:


    ## TODO: Update for python3 and change version


    # Initializes drone attribute and defines last_NDC and flying options
    # return: -
    def __init__(self):
        self.__Version = "3.6.8"

        #Connect to drone
        self.drone = ps_drone_vp3.Drone()
        self.drone.startup()
        self.drone.reset()

        #Wait for reset to complete
        while(self.drone.getBattery()[0] == -1):
            time.sleep(.01)

        print("Battery: " + str(self.drone.getBattery()[0]) + "% \  "+str(self.drone.getBattery()[1]))	# Gives a battery-status

        if self.drone.getBattery()[1] == "empty" :
            sys.exit()

        self.last_NDC = self.drone.NavDataCount

        self.fly_options = {'gliding' : False}

         #For slim, important indicies of the parameter are given
        #All of demo is important*   *maybe
        #none of zimmu_3000 is important because it is unknown
        self.important = {'demo': range(0,12) , 'time' : [0], 'wifi' : [0] , \
                    'magneto': range(0,4) , 'altitude' : range(0,5) , 'pressure_raw': [0,1] , \
                    'wind_speed' : [0 , 1] , 'kalman_pressure':[1] , 'zimmu_3000':[] , \
                    'raw_measures' : range(0,4) , 'phys_measures' : [2 , 3] , 'references' : range(0,5) , \
                    'rc_references' : range(0,5) , 'gyros_offsets' : [0] , 'euler_angles' : [0 ,1] , \
                    'watchdog' : [] , 'trims' : range(0,4) , 'pwm' : range(0,12) , \
                    'state' : []
                    }
        ################################
        #TODO: Finish list of VISION later

        return

    # Testing function
    # return: -
    def main(self , **kwargs):

        options = {'time_lim' : 15 , 'demo' : True , 'desired_data' : ['demo']}
        options.update(kwargs)

        #Determines packet rate
        #True = 15packets/s
        #False = 200pk/s
        self.drone.useDemoMode(options['demo'])
        #and use that to determine the delta time between each packet/frame
        if options['demo']:
            delta_t = 1/ 15
        else:
            delta_t = 1/200


        #Determine which packets to recieve
        self.drone.getNDpackage(options['desired_data'])


        flight_data = self.fly_and_track(options['time_lim'])

        print(flight_data)

        plot_euler_angles.main(flight_data)

        #plot_cartesian.main()

        return


    ## TODO: Have run method set ND packages and if Demo mode etc.
    ##


    # Configures the drone.
    # return: -
    def run(self , **kwargs):

        return

    def thread_fly_and_track(self , time_lim):

        flight_data = []

        flight_thread = Drone_Thread(self.drone , name='flight_thread')
        navdata_thread = Drone_Thread(self.drone , name='navdata_thread')

        threads = [flight_thread , navdata_thread]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        ### TODO: Get data from flight_thread





        return flight_data


    # # TODO: Fix this! it's broken

    # Handles manual flight and packages navdata concurrently
    # return: flight_data - list of time slices of dictionaries of navdata
    def fly_and_track(self , time_lim):
        skip = False

        flight_data = []

        timeout = time.time() + time_lim

        end = False

        print("Ready to fly")
        while not end and time.time() < timeout:
            end  = self.get_key_and_respond()

            if self.drone.NavDataCount != self.last_NDC and not skip:
                skip = True
                self.last_NDC = self.drone.NavDataCount
                _data_slice = self.get_nav_frame()
                flight_data.append(_data_slice)

            if self.drone.NavDataCount != self.last_NDC and skip:
                skip = False
                self.last_NDC = self.drone.NavDataCount

            time.sleep(0.00001)

        self.drone.land()
        while self.drone.NavData["demo"][0][3]:
            time.sleep(0.1)


        return flight_data

    '''
    ----------- Flight section ----------
    '''

    # Allows the user to manually fly for a specified time.
    # return: -
    def manual_flight(self , **kwargs):
        options = {'time_lim':1*60}
        options.update(kwargs)

        timeout = time.time() + options['time_lim']

        #initialize loop variables
        gliding = False
        end = False

        while not end and time.time() < timeout:
            key = self.drone.getKey()
            if key == "p":
                end = True

            elif key == "i":
                self.drone.takeoff()
                while self.drone.NavData["demo"][0][2]:
                    time.sleep(0.1)
            elif key == "k":
                self.drone.land()
                while self.drone.NavData["demo"][0][3]:
                    time.sleep(0.1)

            elif key == " ":
                gliding = not gliding

            elif key == "w":
                self.drone.moveForward()
                time.sleep(.01)
            elif key == "s":
                self.drone.moveBackward()
                time.sleep(.01)

            elif key == "a":
                self.drone.moveLeft()
                time.sleep(.01)
            elif key == "d":
                self.drone.moveRight()
                time.sleep(.01)

            elif key == "q":
                self.drone.turnLeft()
                time.sleep(.01)
            elif key == "e":
                self.drone.turnRight()
                time.sleep(.01)

            elif key == "o":
                self.drone.moveUp()
                time.sleep(.01)
            elif key == "l":
                self.drone.moveDown()
                time.sleep(.01)

            elif key == "":
                if gliding:
                    self.drone.moveForward(0)
                else:
                    self.drone.stop()


            time.sleep(.1)


        self.drone.stop()
        self.drone.land()

        time.sleep(1)

        return


    # Subsection of flight. Simply gets user's key and responds accordingly
    # return: True/False if loop should end
    def get_key_and_respond(self):
        key = self.drone.getKey()
        if key == "p":
            return True

        elif key == "i":
            self.drone.takeoff()
            while self.drone.NavData["demo"][0][2]:
                time.sleep(0.1)
        elif key == "k":
            self.drone.land()
            while self.drone.NavData["demo"][0][3]:
                time.sleep(0.1)

        elif key == " ":
            self.fly_options['gliding'] = not self.fly_options['gliding']

        elif key == "w":
            self.drone.moveForward()
            time.sleep(.02)
        elif key == "s":
            self.drone.moveBackward()
            time.sleep(.02)

        elif key == "a":
            self.drone.moveLeft()
            time.sleep(.02)
        elif key == "d":
            self.drone.moveRight()
            time.sleep(.02)

        elif key == "q":
            self.drone.turnLeft()
            time.sleep(.02)
        elif key == "e":
            self.drone.turnRight()
            time.sleep(.02)

        elif key == "o":
            self.drone.moveUp()
            time.sleep(.02)
        elif key == "l":
            self.drone.moveDown()
            time.sleep(.02)

        elif key == "":
            if self.fly_options['gliding']:
                self.drone.moveForward(0)
            else:
                self.drone.stop()

        return False

    def emergency_landing():
        self.drone.land()

        return

    '''
    ----------- Navdata section ----------
    '''

    # Gets navdata according to arguments and returns nicely packaged data. Can be run from outside this script
    # return: flight_data - list of time slices of dictionaries of navdata
    #         delta_t - change in time between navdata elements
    def get_navdata(self , desired_data , **kwargs):
        options = {'req_take_off' : False , 'demo' : True , 'time_lim' : 4}
        options.update(kwargs)

        #Determines packet rate
        #True = 15packets/s
        #False = 200pk/s
        self.drone.useDemoMode(options['demo'])
        #and use that to determine the delta time between each packet/frame
        if options['demo']:
            delta_t = 1/ 15
        else:
            delta_t = 1/200


        #Determine which packets to recieve
        self.drone.getNDpackage(desired_data)

        #Wait for takeoff
        while options['req_take_off']:
            print("stuck here?")
            if self.drone.State == 1:
                options['req_take_off'] = False
                break
            time.sleep(.5)

        print("about to gather_flight_data")
        flight_data = self.gather_data_set_time(options['time_lim'])
        print("Have gathered data")


        return (flight_data , delta_t)


    #Gathers data as specified by the arguments in main for a specified amount of time
    #return: flight_data - list of time slices of dictionaries of navdata
    def gather_data_set_time(self , time_lim):
        t_end = time.time() + time_lim

        #print(t_end)

        flight_data = []
        last_NDC = self.drone.NavDataCount -1

        while time.time() < t_end:
            #print(time.time())

            while self.drone.NavDataCount == last_NDC:
                #print(self.drone.NavDataCount)
                #if drone.getKey():
                #	end = True
                time.sleep(.00045)

            last_NDC = self.drone.NavDataCount

            _data_slice = self.get_nav_frame()
            flight_data.append(_data_slice)

        return flight_data

    #TODO: Finish list of VISION later

    #Queries drone for navdata after waiting for new data and packages them nicely.
    #Has slim parameter to only take values deemed important. Updates last NDC
    #return: data-  Dictionary of numpy arrays
    def get_nav_frame(self , **kwargs):
        options = {'slim' : True}
        options.update(kwargs)

        data = {}

        if options['slim']:
            for _d_param in self.drone.NavData:

                data[_d_param] = []

                #for important element, copy that
                for _important_elem in self.important[_d_param]:
                    data[_d_param].append(self.drone.NavData[_d_param][_important_elem])
                #data[_d_param] = np.array(drone.NavData[_d_param][_important_elems])

        else:
            for _d_param in self.drone.NavData:
                data[_d_param] = self.drone.NavData[_d_param]
                #data[_d_param] = np.array(drone.NavData[_d_param])

        return data


if __name__ == '__main__':
    #pass

    #Temporary testing

    drone_obj = Chief()
    print("Initialized")
    drone_obj.main()
