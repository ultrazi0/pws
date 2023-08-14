import pygame
from time import sleep

pygame.init()

#joystick = pygame.joystick.Joystick(0)

while True:
    event = pygame.event.wait(500)

    if event.type == pygame.JOYDEVICEREMOVED:
        print('Removed')
        #joystick.quit()

    elif event.type == pygame.JOYDEVICEADDED:
        print('Added')
        print(event.device_index)
        for i in range(pygame.joystick.get_count()):
            print(pygame.joystick.Joystick(i).get_name())
        #joystick.init()
    print('*********')