import RPi.GPIO as gp
from time import sleep, time
from motor import Motor
from controller import buttons, button_names, get_button
from controller import initialize as control_init
from recog import initialize as camera_init, display, cv2, getObjects
from servo import Servo
from stepper import Stepper
from canon import Canon
from configparser import ConfigParser
from math_part import calculate_angle_without_drag, angle_canon


if __name__ == '__main__':
    config = ConfigParser()
    config_file = 'cfg.ini'
    config.read(config_file)

    # Initialize constants
    dx = float(config['constants']['dx'])
    dy = float(config['constants']['dy'])
    r = float(config['constants']['r'])
    last_time_check = -1  # Variable that checks for how long shooting button has been pressed

    gp.setmode(gp.BOARD)
    
    # Initialize DC motors
    m1 = Motor(19, 21, 23, 'l', min_speed=25, min_activation_speed=20)
    m2 = Motor(8, 10, 12, 'r', min_speed=25, min_activation_speed=20)

    # Initialize camera
    control_init()
    cap = camera_init(480, 360, 0)

    # Initialize servo to lift the canon
    phi = 4.447
    alpha0 = 27.818
    start_angle = 36
    servo = Servo(24, lower_border=-2, upper_border=100, min_angle=180-start_angle+alpha0, max_angle=alpha0-start_angle, initial_angle=0)  # Use GPIO numeration

    # Initialize stepper motor to turn the turret
    stepper = Stepper(11, 13, 15, 16, initial_angle=float(config['values']['angle stepper']),
        initial_step=int(config['values']['step']), upper_border=120, lower_border=-120, config=config, config_file=config_file, turret=True)
    
    # Initialize canon
    canon = Canon(3, 5, 7, 36, config)

    # Main loop
    print('Hello, dear user! What would you like to annihilate today?\n')
    while 1:
        try:
            x, y = map(float, input('Enter the coordinates: ').split())
            print('Coordinates read')
            
            angle = calculate_angle_without_drag(x, y, float(config['constants']['initial velocity']), 
                x0=float(config['constants']['x0']), y0=float(config['constants']['y0']))
            alpha = angle_canon(angle, dx, dy, r)
            servo.rotate_to(alpha)
            sleep(1.5)
            print('Aim complete')

            input('Press Enter to fire...')
            canon.shoot()

            sleep(0.5)
            print('Target annihilated, awaiting new instructions')
            
            print('*********')
            sleep(0.01)
        except KeyboardInterrupt:
            servo.kill()
            gp.cleanup()
            cv2.destroyAllWindows()
            print("\nGoodbye!")
            break
