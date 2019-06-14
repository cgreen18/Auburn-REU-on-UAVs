'''
Title: debug_nav.py
Author: Conor
Description: Hard coded script to print out certain navdata parameters
Usage: Run through main
Version:
1.0 - June 13 2019 - Created
'''

import ps_drone
import time

import numpy

drone = ps_drone.Drone()
drone.startup()
drone.reset()
while(drone.getBattery()[0] == -1): time.sleep(.01)
drone.useDemoMode(True)
#parameters
drone.getNDpackage(["magneto"])

my_NDC = drone.NavDataCount

for i in range(0,1000):
    print("\nPrinting data point: $d" % i)
    print()
