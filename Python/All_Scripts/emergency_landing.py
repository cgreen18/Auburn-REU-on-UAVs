'''
Title: emergency_landing.py
Author: Conor Green
Description: Simple script to land the drone when another script has lost connection
Usage: Run as main through command line
Version:
1.0 - June 20 2019 - Created and final
'''

import Chief_Drone

print("Emergency!")

drone_obj = Chief_Drone.Chief()
drone_obj.emergency_landing()

print("Disaster avoided?")
