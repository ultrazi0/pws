from time import sleep
from motors import Servo, Stepper
from configparser import ConfigParser
from math_part import angle_turret, angle_cannon


def aim(angles, stepper, servo, config, delay=None):
    angle_x, angle_y = angles

    # Horizontal angle
    stepper_turn_angle_x = angle_turret(angle_x, config=config)
    stepper.turn_by(stepper_turn_angle_x)
    
    # Vertical angle
    dx = float(config['constants']['dx'])
    dy = float(config['constants']['dy'])
    r = float(config['constants']['r'])

    turn_angle_y = angle_cannon(angle_y, dx=dx, dy=dy, r=r, give_degrees=True)  # Calculate the turn angle
    servo.rotate_to(turn_angle_y)  # Rotate

    if delay is not None:
        sleep(delay)


if __name__ == "__main__":
    import RPi.GPIO as gp
    
    # Initialize config
    config = ConfigParser()
    config_file = 'cfg.ini'
    config.read(config_file)

    gp.setmode(gp.BOARD)  # Set the pin-numbering

    # Initialize servo to lift the cannon
    dy = float(config['constants']['dy'])
    r = float(config['constants']['r'])
    start_angle = 52  # Angle between the servo's "original" zero-position 
                      # and the line that goes through the certer of the disk and is parallel to cannon's zero-angle
                      # this parameter is used to set up the zero-position of the cannon correctly
    
    servo = Servo(24, dy=dy, r=r, start_angle=start_angle, initial_angle=0)  # Use GPIO numeration

    # Initialize stepper motor to turn the turret
    initial_angle = float(config['values']['angle stepper'])
    initial_step = int(config['values']['step'])

    stepper = Stepper(11, 13, 15, 16, initial_angle=initial_angle,
        initial_step=initial_step, upper_border=120, lower_border=-120, config=config, config_file=config_file)
    
    try:
        while 1:
            angle = float(input('Enter angle: '))

            aim((angle, 0), stepper, servo, config)
    except KeyboardInterrupt:
        print("Closed via Ctrl + C")
    finally:
        servo.kill()
        gp.cleanup()
        print("\nGoodbye!")
    