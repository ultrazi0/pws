import pygame
from time import sleep, time

"""
Axes:
0: Left stick x
1: Left stick y
2: Right stick x
3: Right stick y
5: Right trigger
6: Left trigger
"""

button_names = {
    0: 'a',
    1: 'b',
    3: 'x',
    4: 'y',
    6: 'lb',
    7: 'rb',
    11: 'opt',
    13: 'ls',
    14: 'rs',
    15: 'map',
    16: 'xbox'
}

joy_hat_names = {
    0: 'D-pad-X',
    1: 'D-pad-Y'
}

buttons = {
    'a': 0, 'b': 0, 'x':0, 'y': 0, 'lb': 0, 'rb': 0, 'opt': 0, 'ls': 0, 'rs': 0, 'map': 0, 'xbox': 0, 'D-pad-X': 0, 'D-pad-Y': 0, 
    'axis0': 0., 'axis1': 0., 'axis2': 0., 'axis3': 0., 'axis4': 0., 'axis5': -1.
}
axes = [0., 0., 0., 0., -1., -1.]

def wait_for_a_controller(duration=None, function=None):
    start = time()
    while 1:
        if duration is not None:
            if time()-start > duration:
                return None
        
        # Wait for event
        event = pygame.event.wait(500)

        if function:
            # If there is a function to execute while waiting,
            # execute it
            function()

        if event.type == pygame.JOYDEVICEADDED:
        # If a joystick was added, return its ID
            return event.device_index

def initialize(controller_number=0):
    controller = pygame.joystick.Joystick(controller_number)  
    controller.init()

    return controller

def get_button(button='', joystick_deadzone=0.02):
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            if abs(event.value) > joystick_deadzone:
                axes[event.axis] = round(event.value, 2)
            else:
                axes[event.axis] = 0
        elif event.type == pygame.JOYBUTTONDOWN:
            buttons[button_names[event.button]] = 1
        elif event.type == pygame.JOYBUTTONUP:
            buttons[button_names[event.button]] = 0
        elif event.type == pygame.JOYHATMOTION:
            buttons[joy_hat_names[0]] = event.value[0]
            buttons[joy_hat_names[1]] = event.value[1]


    for i, value in enumerate(axes):
        ax_name = 'axis' + str(i)
        buttons[ax_name] = value
    
    if button:
        return buttons[button]
    return buttons


if __name__ == "__main__":

    pygame.init()
    device_id = wait_for_a_controller()

    controller = initialize(device_id)

    try:
        while 1:
            control_values = get_button()
            print(control_values)

            sleep(0.05)
    except KeyboardInterrupt:
        print('\nClosed vin Ctrl + C')
    finally:
        print('\nGoodbye!')