'''
Title: drone_flying.py
Author: Conor Green
'''
###TODO: Implement better or more efficient user input than pygame

import ps_drone

def main():
    drone = initialize_drone()

    drone.takeoff()

    end = False
    while not end:
        key = drone.getKey()
        if key == " ":      drone.moveForward(0)
        elif key == "w":	drone.moveForward()
        elif key == "s":	drone.moveBackward()
		elif key == "a":	drone.moveLeft()
		elif key == "d":	drone.moveRight()
		elif key == "q":	drone.turnLeft()
		elif key == "e":	drone.turnRight()
        elif key != "":     end = True

        time.sleep(.1)

    drone.stop()


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
