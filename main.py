import cv2
import pygame
import numpy as np
import RPi.GPIO as GPIO
from gpiozero import LED
from time import sleep, time
from motor import Motor_PWM
from controller import buttons, button_names, get_button, wait_for_a_controller
from controller import initialize as control_init
from camera import process_capture, process_show, read_image_from_shared_memory
from servo import Servo
from stepper import Stepper
from canon import Canon
from configparser import ConfigParser
from aim import get_aim, aim
from math_part import find_middle, translate_origin_to_canon, translate_image_point, sin, cos, find_angle_with_drag, radians, pi
from qrcode import get_qrcode_pyzbar, get_average_coordinates_from_array_of_images
from distance_qr import distance
from multiprocessing import Process, Event
from multiprocessing.shared_memory import SharedMemory
from CameraCalibration.undistort import undistort, get_calib_params


def start_aiming(image=None, images_array=None):
    assert (image is not None) or (images_array is not None)

    #cv2.imwrite('crap0.jpg', image)
    if image is not None:
        image = undistort(image, cmx, dist)
        h, w = image.shape[:2]

        # Rotate the image, because the camera actually sees an inverted image
        image = cv2.flip(image, 0)
        
        qr = list(get_qrcode_pyzbar(image))
        coords = qr[0][0] if qr else np.array([]) # Get the coordinates
    
    if images_array is not None:
        # Rotate all images 180 degrees
        images_array = np.array([cv2.flip(undistort(img, cmx, dist), 0) for img in images_array], dtype=np.uint8)

        h, w = images_array[0].shape[:2]

        # Get the coordinates
        coords = get_average_coordinates_from_array_of_images(images_array)
        print(coords)

    if coords.size:
        # Translate the "image" coordinates to "real" coordinates (with center of the camera as origint)
        real_points = [translate_image_point(point, current_resolution=(w, h)) for point in coords]
        print(real_points)

        # Draw borders around the QR-code and show the frame in separate window
        # middle = find_middle(coords, give_int=True)
        # print(middle, 'ff')
        # image = cv2.polylines(image, [coords], 1, (0, 255, 0), 2)
        # cv2.circle(image, middle, 2, (0, 0, 255), -1)
        # cv2.circle(image, (320, 240), 3, (255, 0, 0), -1)
        
        # # Show the image with the target
        # cv2.imshow('Target', cv2.rotate(image, cv2.ROTATE_180))
        # cv2.circle(image, tuple(coords[0]), 5, (255, 0, 0), -1)
        # cv2.circle(image, tuple(coords[1]), 5, (0, 0, 255), -1)
        # cv2.circle(image, tuple(coords[2]), 5, (0, 255, 255), -1)
        # cv2.imwrite('crap.jpg', image)

        ###########################

        # Calculate the distance from camera
        d_camera, M_coords = distance(real_points, focus=focus_length, qrcode_length=qr_width, image_size=image_resol, return_middle=True)

        # Print it out
        print('Distance camera:', d_camera)

        # Calculate the distance and the angles from the center of the turret
        angle_turret_x, angle_turret_y, d_turret = translate_origin_to_canon(d_camera, M_coords, coordinates_of_camera_with_respect_to_the_turret)

        # Calculate the distance and angles from the canon, only vertical angle needed
        angle_canon_x, angle_canon_y, d_canon = translate_origin_to_canon(d_camera, M_coords, coordinates_of_camera_with_respect_to_the_canon)

        # Horizontal angle
        angle_x = angle_turret_x

        # Vertical angle
        x_distance = d_canon * cos(radians(angle_canon_y))
        y_distance = d_canon * sin(radians(angle_canon_y))

        print('an', angle_canon_y)
        angle_y = find_angle_with_drag(x_distance, y_distance, mass, initial_velocity, coefficient, g)

        # Print everything out
        print("Distance canon", d_canon)
        print('x-angle:', angle_x)
        print('y-angle:', angle_y)
        
        # Aim
        aim((angle_x, angle_y), stepper, servo, config, delay=1)
    else:
        print('Nothing, Sir!')
    
    print("We're done, Sir!")




if __name__ == '__main__':
    # Initialize config
    config = ConfigParser()
    config_file = 'cfg.ini'
    config.read(config_file)

    # Set GPIO numeration
    GPIO.setmode(GPIO.BOARD)

    # Projectile-related constants
    mass = 0.12*10**(-3)
    initial_velocity = float(config['constants']['initial velocity'])
    coefficient = 1/2*1.225*0.47*pi*(0.006/2)**2
    g = 9.8122

    # Camera-related constants
    coordinates_of_camera_with_respect_to_the_turret = (0.035, -0.03, 0.04)  # Center of the turret, needed for correct horizontal angle
    coordinates_of_camera_with_respect_to_the_canon = (0.05, -0.09, 0.019)  # Canon itself, needed for correct vertical angle

    focus_length = 3.04*10**(-3)
    qr_width = 0.122
    image_resol = (640, 480)

    cmx, dist = get_calib_params('CameraCalibration/calibration.pkl')

    # Initialize camera-related objects
    img_mem = SharedMemory(name='ImageMemory', create=True, size=2**22)  # Used to share images between processes
    closed = Event()  # Needed to indicate that processes should close 

    # Process that captures and sends images
    capture = Process(target=process_capture, args=(image_resol, closed))
    # Process taht reads and shows images
    show = Process(target=process_show, args=(image_resol, closed))


    # Initialize servo to lift the canon
    dy = float(config['constants']['dy'])
    r = float(config['constants']['r'])
    start_angle = 52  # Angle between the servo's "original" zero-position 
                      # and the line that goes through the certer of the disk and is parallel to canon's zero-angle
                      # this parameter is used to set up the zero-position of the canon correctly
    
    servo = Servo(24, dy=dy, r=r, start_angle=start_angle, initial_angle=0)  # Use GPIO numeration

    # Initialize stepper motor to turn the turret
    initial_angle = float(config['values']['angle stepper'])
    initial_step = int(config['values']['step'])

    stepper = Stepper(11, 13, 15, 16, initial_angle=initial_angle,
        initial_step=initial_step, upper_border=120, lower_border=-120, config=config, config_file=config_file)
    
    # Initialize DC motors
    m1 = Motor_PWM(21, 19, 'l', min_speed=20, min_activation_speed=20)
    m2 = Motor_PWM(8, 10, 'r', min_speed=20, min_activation_speed=20)
       
    # Initialize canon
    canon = Canon(5, 3, 7, 36, config)

    # Initialize LEDs
    last_time_a_button_was_pressed = -1
    front_led = LED('BOARD23')  # Use gpiozero numeration

    # Main loop
    try:
        # Start processes
        capture.start()
        show.start()

        # Initialize pygame
        pygame.init()

        # Wait for a controller
        control_id = wait_for_a_controller(function=front_led.toggle)
        front_led.on()
        
        # Initialize controller
        controller = control_init(control_id)

        while 1:
            # Read the inputs from the controller
            control_values = get_button(joystick_deadzone=0.08)

            # DC motors
            m1.move(-control_values['axis1'], control_values['axis0'])
            m2.move(-control_values['axis1'], control_values['axis0'])

            # The turret
            servo.rotate_by(-control_values['axis3'], do_not_use_sleep=True)  # Up/Down
            stepper.rotate(control_values['axis2'], turn_coefficient=50, delay=0.002)  # Left/Right

            # LEDs
            if control_values['D-pad-Y'] == 1:
                if time() - last_time_a_button_was_pressed > 0.25:
                    front_led.toggle()
                last_time_a_button_was_pressed = time()

            # Shooting
            if control_values['x']:
                if canon.check_command():
                    sleep(0.5)  # To ensure nothing else is moving
                    canon.shoot()
            else:
                canon.last_time_check = -1
            
            # Aiming
            if control_values['lb']:
                # If LB or Space is pressed

                img = read_image_from_shared_memory(image_resol, n=1)
                start_aiming(images_array=img)
            
            if closed.is_set():
                print('Closed via Q-button')
                break

            sleep(0.01)
    except KeyboardInterrupt:
        print('\nCommand was received to shut down...')
    finally:
        img_mem.close()
        img_mem.unlink()
        servo.kill()
        front_led.close()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print("\nGoodbye!")
