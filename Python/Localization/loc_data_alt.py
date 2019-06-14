'''
Title: map_nav
Author: Conor Green

'''

#Recommended imports
import time
#import sys
import ps_drone

#Extended imports
import matplotlib.pyplot as plt
import numpy as np

def main():

	drone = ps_drone.Drone()
	drone.startup()

	drone.reset()

	while(drone.getBattery()[0] == -1): time.sleep(.01)   #Wait until done resetting

	drone.useDemoMode(False)
	drone.getNDpackage(["altitude","demo","magneto"])

	my_NDC = drone.NavDataCount

	num_pts = 3000
	alt_data = np.zeros( (3,num_pts) )

	#plt.figure()
	#plt.plot([1,2,3,4])
	#plt.show()

	for i in range(0,num_pts):
		while drone.NavDataCount == my_NDC:
			#if drone.getKey():
			#	end = True
			time.sleep(.001)
		my_NDC = drone.NavDataCount

		print('Getting data point' + str(i) )
		print('--------------------')

        for d_type in drone.NavData:
            print(str(type(d_type)))

		alt_data[0][i] = drone.NavData['altitude'][0]
		#print(alt_data[0][i])
		alt_data[1][i] = drone.NavData['altitude'][2]
		alt_data[2][i] = drone.NavData['altitude'][3] #-500

	plt.figure()
	plt.plot( range(0,num_pts) , alt_data[0][:] )
	plt.plot( range(0,num_pts) , alt_data[1][:] )
	plt.plot( range(0,num_pts) , alt_data[2][:]  )
	plt.legend()
	plt.show()

	time.sleep(3)

	return

if __name__ == '__main__':
	main()
