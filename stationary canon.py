import RPi.GPIO as gp
from time import sleep, time
from recog import initialize as camera_init, display, cv2, getObjects
from servo import Servo
from stepper import Stepper
from canon import Canon
from configparser import ConfigParser
from aim import get_aim, aim
from qrcode import get_qrcode
from math_part import average, find_angle_x, find_angle_y_laser, angle_canon, angle_turret
from math import atan, degrees


def main():

    target = tuple(map(int, input('Enter the coorditates of the target: ').split()))  # (X, Y, Z) <==> (offset, distance, hight)
    print("--> Coordinates read")


    angle_x = degrees(atan(target[0]/target[1]))
    angle_y = degrees(atan(target[2]/target[1]))  # FIXME: no drag
    print(f"--> Aim calculated: {angle_x}, {angle_y}")

    # Angle transformation
    # angle_x = angle_turret(angle_x, config=config)
    angle_y = angle_canon(angle_y, dx=42.5, dy=3.5, r=7.5)

    aim((angle_x, angle_y), stepper, servo, delay=1)
    print('--> Aiming done')

    print('--> Prepairing to shoot')
    sleep(0.5)  # To ensure nothing else is moving
    
    canon.shoot()
    print("We're done, Sir!")


if __name__ == '__main__':
    # Initialize config
    config = ConfigParser()
    config_file = 'cfg.ini'
    config.read(config_file)

    # Initialize constants
    gp.setmode(gp.BOARD)

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
    try:
        while 1:
            main()
            print("**************************")
            sleep(1)
    except KeyboardInterrupt:
        servo.kill()
        gp.cleanup()
        print("\nGoodbye!")
            
