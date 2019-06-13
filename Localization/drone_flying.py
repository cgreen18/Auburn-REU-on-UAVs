'''
Title: drone_flying.py
Author: Conor Green
'''
###TODO: Implement better or more efficient user input

import ps_drone
import pygame

def main(**kwargs):
    options = {'smooth' : False}
    options.update(kwargs)

    drone = initialize_drone()
    print('Initialized')

    if options['smooth']:
        smooth_flying()
    else:
        rough_flying()

    print('Finished Flying')

    return

def smooth_flying():
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

        if keys[pygame.SPACE]:
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

        drone.move(LR,BF,DU,rot_LR)


    drone.stop()
    drone.land()

    time.sleep(1)

    return

def rough_flying():

    drone.takeoff()

    end = False
    while not end:
        key = drone.getKey()
        if key == " ":
            drone.moveForward(0)
        elif key == "w":
            drone.moveForward()
        elif key == "s":
            drone.moveBackward()
        elif key == "a":
            drone.moveLeft()
        elif key == "d":
            drone.moveRight()
        elif key == "q":
            drone.turnLeft()
        elif key == "e":
            drone.turnRight()
        elif key == "":
            drone.stop()
        elif key != "":
            end = True

        time.sleep(.1)


    drone.stop()
    drone.land()

    time.sleep(1)

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
