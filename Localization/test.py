#import ps_drone
import time
import drone_slam

drone = drone_slam.initialize_drone()
print('initialized')
time.sleep(1)
a = drone.State
print(a)
