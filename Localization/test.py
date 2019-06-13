#import ps_drone
import time
import drone_slam
import pygame

#drone = drone_slam.initialize_drone()
print('initialized')
time.sleep(1)
#a = drone.State
#print(a[0])

pygame.init()

while True:

    keys = pygame.key.get_pressed()
    screen = pygame.display.set_mode((640,480))

    if keys[pygame.K_RIGHT] and keys[pygame.K_LEFT]:
        print(keys[pygame.K_RIGHT])
        print('Both pressed')
    if keys[pygame.K_DOWN]:
        print(keys[pygame.K_RIGHT])
    if keys[pygame.K_UP]:
        print('break')
        break
