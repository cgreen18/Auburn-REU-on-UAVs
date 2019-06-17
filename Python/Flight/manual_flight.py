'''
Title: manual_flight.py
Author: Conor Green
Description: User flies the drone. Has smooth and rough flying modes
Usage: Run as main or through another program
Version:
1.0 - June 13 2019 - Rough Draft Complete (broken)
'''
###TODO: Implement better or more efficient user input

###TODO: Doesn't respond to commands!

###TODO:Parse arguments with arg parser



import ps_drone


import pygame
import time

def main(**kwargs):

    options = {'smooth' : False , 'time_lim' : 30}
    options.update(kwargs)

    drone = ps_drone.Drone()
    drone.startup()
    drone.reset()

    while(drone.getBattery()[0] == -1): time.sleep(.01)

    print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
    if drone.getBattery()[1] == "empty":
        return

    print('Initialized')

    if options['smooth']:
        smooth_flying(drone , options['time_lim'])
    else:
        rough_flying(drone , options['time_lim'])

    print('Finished Flying')

    return

def smooth_flying(drone , time_lim):

    #broken
    return

    pygame.init()
    screen = pygame.display.set_mode((640,480))

    drone.takeoff()

    end = False

    LR = 0
    FB = 0
    rot_LR = 0
    UD = 0

    landed = False

    while not end:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE]:
            end = True
            break

        if keys[pygame.K_SPACE]:
            if not landed:
                landed = True
                drone.land()
                time.sleep(.2)
            else:
                landed = False
                drone.takeoff()
                time.sleep(.2)

        #Left - Right
        _ = keys[pygame.K_d] - keys[pygame.K_a]
        LR = 0.8*LR + 0.2*_

        #Forward - Back
        _ = keys[pygame.K_w] - keys[pygame.K_s]
        FB = 0.8*FB + 0.2*_

        #Rotation (yaw) Left - Right
        _ = keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]
        rot_LR = 0.8*rot_LR + 0.2*_

        #Up - Down
        _ = keys[pygame.K_UP] - keys[pygame.K_DOWN]
        UD = 0.8*UD + 0.2*_

        drone.move(LR,FB,UD,rot_LR)


    drone.stop()
    drone.land()

    time.sleep(1)

    return

def rough_flying(drone , time_lim):

    timeout = time.time() + time_lim

    gliding = False
    end = False
    while not end and time.time() < timeout:
        key = drone.getKey()
        if key == "p":
            end = True
        elif key == "i":
            drone.takeoff()
            while drone.NavData["demo"][0][2]:
                time.sleep(0.1)
        elif key == "k":
            drone.land()
            while drone.NavData["demo"][0][3]:
                time.sleep(0.1)
        elif key == " ":
            gliding = not gliding
        elif key == "w":
            drone.moveForward()
            time.sleep(.05)
        elif key == "s":
            drone.moveBackward()
            time.sleep(.05)
        elif key == "a":
            drone.moveLeft()
            time.sleep(.05)
        elif key == "d":
            drone.moveRight()
            time.sleep(.05)
        elif key == "q":
            drone.turnLeft()
            time.sleep(.05)
        elif key == "e":
            drone.turnRight()
            time.sleep(.05)
        elif key == "o":
            drone.moveUp()
            time.sleep(.05)
        elif key == "l":
            drone.moveDown()
            time.sleep(.05)
        elif key == "":
            if gliding:
                drone.moveForward(0)
            else:
                drone.stop()


        time.sleep(.1)


    drone.stop()
    drone.land()

    time.sleep(1)

    return


if __name__ == '__main__':
    main()
