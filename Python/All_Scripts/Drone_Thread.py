'''
Title: Drone_Thread.py
Author: Conor Green
Description: A custom threading class (inherits threading.Thread) that shares drone object between threads
Usage: Instantiate and run through Chief_Drone.thread_fly_and_track()
Version:
1.0 - June 20 2019 - Created custom threading class
'''
import threading
import time

import Chief_Drone

class Drone_Thread(threading.Thread):

    def __init__(self , drone , which , time_lim , *args , **kwargs):
        super(Drone_Thread , self).__init__(*args , **kwargs)
        self.drone = drone
        print(drone)
        self.which = which
        self.time_lim = time_lim

    def run(self ):
        if self.which == 'fly':
            self.fly(self.time_lim)
        elif self.which == 'navdata':
            self.get_navdata(self.time_lim)

        return

    def fly(self , time_lim):
        timeout = time.time() + time_lim

        end = False

        while not end and time.time() < timeout:
            end = self.drone.get_key_and_respond()

        return

    def get_navdata(self, time_lim):

        flight_data = self.drone.gather_data_set_time(time_lim)

        print("Printing flight data")
        print(flight_data)

        return
        #return flight_data


if __name__ == '__main__':
    pass
