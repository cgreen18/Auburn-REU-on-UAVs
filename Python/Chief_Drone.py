'''
Title: Chief_Drone.py
Author: Conor Green and Brenden Stevens
Description: Class to maintain the same drone object (from ps_drone). Holds methods to get navdata and fly
Usage: Call from principal_drone (or main at the moment) to multiprocess navdata and flying
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

#Temporary imports
import temp_print_flight_data


### TODOs: see __init__
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

        self.flight_data = []
        self.parallel_time_stamp = []



        #Defaults
        self.options = {'time_lim' : 15 , 'demo_mode' : True , 'desired_data' : ['demo']}

        self.delta_t = 1/15

        return

    # Configures the drone and Chief_Drone object
    # return: -
    def run(self , **kwargs):
        self.options.update(kwargs)

        #Determines packet rate
        #True = 15packets/s
        #False = 200pk/s
        self.drone.useDemoMode(self.options['demo_mode'])
        #and use that to determine the delta time between each packet/frame
        if self.options['demo_mode']:
            self.delta_t = 1/ 15
        else:
            self.delta_t = 1/200


        print(self.options['desired_data'])

        #Determine which packets to recieve
        self.drone.getNDpackage(self.options['desired_data'])

        self.main()

        return

    #Main functionality of Chief_Drone: to fly AND track the navdata of the drone. Can add plotting functions here
    def main(self):

        self.thread_fly_and_track(self.options['time_lim'])

        #self.drone.takeoff()
        #print("Taking Off")
        #time.sleep(1)

        #self.gather_data_set_time(self.options['time_lim'])

        #self.drone.land()
        #print("Landing")
        #time.sleep(5)

        #print(self.flight_data)

        #temp_print_flight_data.main(self.flight_data)

        plot_cartesian.main(self.flight_data , dt = self.delta_t)

        #plot_euler_angles.main(self.flight_data , sleeptime = 0.00001)

        #name = "position_data.txt"
        #self.special_print_position(name)

        #name = "attitude_data.txt"
        #self.special_print_euler(name)

        # name = "pos_and_eul_data.txt"
        # self.special_print_eul_and_pos(name)



        return

    def thread_fly_and_track(self , time_lim):

        #Initialize threads
        flight_thread = Drone_Thread.Drone_Thread(self , 'fly' , time_lim , name='flight_thread')

        navdata_thread = Drone_Thread.Drone_Thread(self , 'navdata' , time_lim , name='navdata_thread')

        threads = [flight_thread , navdata_thread]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        return


    # Handles manual flight and packages navdata concurrently. Updates self.flight_data
    # return:
    def fly_and_track(self , time_lim):
        skip = False

        timeout = time.time() + time_lim

        end = False

        print("Ready to fly")
        while not end and time.time() < timeout:
            end  = self.get_key_and_respond()

            if self.drone.NavDataCount != self.last_NDC and not skip:
                skip = True
                self.last_NDC = self.drone.NavDataCount
                _data_slice = self.get_nav_frame()
                self.flight_data.append(_data_slice)

            if self.drone.NavDataCount != self.last_NDC and skip:
                skip = False
                self.last_NDC = self.drone.NavDataCount

            time.sleep(0.00001)

        self.drone.land()
        while self.drone.NavData["demo"][0][3]:
            time.sleep(0.1)


        return

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

    def emergency_landing(self):
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
        self.gather_data_set_time(options['time_lim'])
        print("Have gathered data")


        return (flight_data , delta_t)


    #Gathers data as specified by the arguments in main for a specified amount of time. Updates self.flight_data to the flight data!
    def gather_data_set_time(self , time_lim):
        t_end = time.time() + time_lim

        last_NDC = self.drone.NavDataCount -1

        while time.time() < t_end:

            while self.drone.NavDataCount == last_NDC:
                time.sleep(.0004)

            last_NDC = self.drone.NavDataCount

            _data_slice = self.get_nav_frame()
            self.flight_data.append(_data_slice)
            self.parallel_time_stamp.append(time.time())

        return

    #Gathers data as specified by the arguments in main for a specified amount of time. Updates self.flight_data to the flight data and prints certain data per gather as specified in args
    def gather_data_set_time_and_print(self , time_lim , print_what):
        t_end = time.time() + time_lim

        last_NDC = self.drone.NavDataCount -1

        while time.time() < t_end:

            while self.drone.NavDataCount == last_NDC:
                time.sleep(.0004)

            last_NDC = self.drone.NavDataCount

            _data_slice = self.get_nav_frame()
            self.flight_data.append(_data_slice)
            self.parallel_time_stamp.append(time.time())
            
        return

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

    '''
    ----------- Exporting data section ----------
    '''
    def special_print_eul_and_pos(self , name):
        vel_data = self.parse_flight_into_vel()
        pos_data = self.handle_vel_data( vel_data )

        eul_data = self.parse_flight_into_eul_and_convert()

        with open(name , "w") as file:

            num_pts = len(self.flight_data)


            #skip i = 1 to skip pos_data[0] = np.zeros(3,1)
            for i in range(1,num_pts):

                eul_t_slice = eul_data[:,i]
                pos_t_slice = pos_data[:,i]

                t_stamp = self.parallel_time_stamp[i]

                string = str(t_stamp) + ":"

                for elem in eul_t_slice:
                    string += str(elem) + ":"

                for elem in pos_t_slice:
                    string += str(elem) + ":"

                string = string[:-1]
                string += "\n"
                file.write(string)

            file.close()

        return

    ##TODO: THIS IS COPY PASTE, DOESNT WORK
    def special_print_euler(self , name):

        vel_data = self.parse_flight_into_vel()
        pos_data = self.handle_vel_data( vel_data )

        with open(name , "w") as file:

            num_pts = len(self.flight_data)


            #skip i = 1 to skip pos_data[0] = np.zeros(3,1)
            for i in range(1,num_pts):

                pos_t_slice = pos_data[:,i]

                t_stamp = self.parallel_time_stamp[i]

                string = str(t_stamp) + ":"
                for elem in pos_t_slice:
                    string += str(elem) + ":"

                string = string[:-1]
                string += "\n"
                file.write(string)

            file.close()

        return

    def special_print_position(self , name):

        vel_data = self.parse_flight_into_vel()
        pos_data = self.handle_vel_data( vel_data )

        with open(name , "w") as file:

            num_pts = len(self.flight_data)


            #skip i = 1 to skip pos_data[0] = np.zeros(3,1)
            for i in range(1,num_pts):

                pos_t_slice = pos_data[:,i]

                t_stamp = self.parallel_time_stamp[i]

                string = str(t_stamp) + ":"
                for elem in pos_t_slice:
                    string += str(elem) + ":"

                string = string[:-1]
                string += "\n"
                file.write(string)

            file.close()

        return

    #Parses self.flight_data into euler angles as np array
    #return: euler angles
    def parse_flight_into_eul_and_convert(self):

        euler_angles = [ dict['demo'][2] for dict in flight_data]
        euler_angles_numpy = np.array(euler_angles)

        return euler_angles_numpy

    #Parses self.flight_data into velocity
    #return: velocities
    def parse_flight_into_vel(self):

        velocity_data = [ dict['demo'][4] for dict in flight_data ]

        return velocity_data

    #Turns velocity data into position data. Adjusts for original position if guesstimate option is True.
    #return: positions
    def handle_vel_data( self, velocities ):

        pos = np.zeros((3,1))

        for vel in velocities:

            delta_pos = self.calc_delta_pos(vel )
            indx_last_t_slice = pos.shape[1] -1

            new_pos = pos[:,indx_last_t_slice].reshape(3,1) + delta_pos

            pos = np.append( pos , new_pos , axis=1 )

        return pos

    def calc_delta_pos(self, vel):
        vel_np = np.array(vel)
        delta_pos = vel_np * self.delta_t
        delta_pos = delta_pos.reshape((3,1))
        return delta_pos


if __name__ == '__main__':
    #pass

    #Temporary testing

    drone_obj = Chief()
    print("Initialized")
    drone_obj.run(time_lim = 45  , desired_data = ['demo' ,'magneto','altitude', 'raw_measures' , 'references'] , demo_mode = False)
    #drone_obj.run(time_lim=3 , desired_data = ['demo'] , demo_mode = True)
