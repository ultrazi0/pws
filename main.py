import cv2
import pygame
import numpy as np
import RPi.GPIO as GPIO
from turret.cannon import Cannon
from gpiozero import LED
from motors import Motor_PWM, Servo, Stepper
from time import sleep, time
from turret.aim import aim
from turret.distance_qr import distance
from configparser import ConfigParser
from controller import initialize as control_init
from multiprocessing import Process, Event, Value
from multiprocessing.shared_memory import SharedMemory
from CameraCalibration.undistort import undistort, get_calib_params
from controller import buttons, button_names, get_button, wait_for_a_controller
from camera.camera import process_capture, process_show, read_image_from_shared_memory
from camera.qrcode import get_qrcode_pyzbar, get_average_coordinates_from_array_of_images
from math_part import translate_origin_to_cannon, translate_image_point, sin, cos, find_angle_with_drag, radians, memory_size, pi
from soundeffects import *


def start_aiming(image=None, images_array=None):
    assert (image is not None) or (images_array is not None)

    success = None

    #cv2.imwrite('crap0.jpg', image)
    if image is not None:
        # Undistort the image
        image = undistort(image, cmx, dist)
        h, w = image.shape[:2]

        # Rotate the image, because the camera actually sees an inverted image
        image = cv2.flip(image, 0)
        
        qr = list(get_qrcode_pyzbar(image))
        coords = qr[0][0] if qr else np.array([]) # Get the coordinates
    
    if images_array is not None:
        # Undistort and rotate all images 180 degrees
        images_array = np.array([cv2.flip(undistort(img, cmx, dist), 0) for img in images_array], dtype=np.uint8)

        h, w = images_array[0].shape[:2]

        # Get the coordinates
        coords = get_average_coordinates_from_array_of_images(images_array)

    if coords.size:
        # Translate the "image" coordinates to "real" coordinates (with center of the camera as origint)
        real_points = [translate_image_point(point, current_resolution=(w, h)) for point in coords]

        ###########################

        # Calculate the distance from camera
        d_camera, M_coords = distance(real_points, focus=focus_length, qrcode_length=qr_width, image_size=image_resol, return_middle=True)

        # Print it out
        print('Distance camera:', d_camera)

        # Calculate the distance and the angles from the center of the turret, only horizontal angle needed
        angle_turret_x, angle_turret_y, d_turret = translate_origin_to_cannon(d_camera, M_coords, coordinates_of_camera_with_respect_to_the_turret)

        # Calculate the distance and angles from the cannon
        angle_cannon_x, angle_cannon_y, d_cannon = translate_origin_to_cannon(d_camera, M_coords, coordinates_of_camera_with_respect_to_the_cannon)

        # Horizontal angle
        angle_x = angle_turret_x

        # Vertical angle
        x_distance = d_cannon * cos(radians(angle_cannon_y))
        y_distance = d_cannon * sin(radians(angle_cannon_y))

        print('an', angle_cannon_y)
        angle_y = find_angle_with_drag(x_distance, y_distance, mass, initial_velocity, coefficient, g)

        # Print everything out
        print("Distance cannon", d_cannon)
        print('x-angle:', angle_x)
        print('y-angle:', angle_y)
        
        # Aim
        aim((angle_x, angle_y), stepper, servo, config, delay=1)

        success = True
    else:
        print('Nothing, Sir!')
        success = False
    
    print("We're done, Sir!")
    return success


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
    coordinates_of_camera_with_respect_to_the_cannon = (0.05, -0.09, 0.019)  # Cannon itself, needed for correct vertical angle

    focus_length = 3.04*10**(-3)
    qr_width = 0.122
    image_resol = (640, 480)

    cmx, dist = get_calib_params('CameraCalibration/calibration.pkl')  # Parameters for camera calibration

    # Initialize camera-related objects
    mem_size = memory_size(image_resol)  # Calculate the memory size needed for that resolution

    img_mem = SharedMemory(name='ImageMemory', create=True, size=mem_size)  # Used to share images between processes
    closed = Event()  # Needed to indicate that processes should close 
    
    # Shared float value to show current turret angle
    shared_turret_angle = Value('f')

    # Process that captures and sends images
    capture = Process(target=process_capture, args=(image_resol, closed))
    # Process taht reads and shows images
    show = Process(target=process_show, args=(image_resol, closed, shared_turret_angle))


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
    
    # Initialize DC motors
    m1 = Motor_PWM(21, 19, 'l', min_speed=20, min_activation_speed=20)
    m2 = Motor_PWM(8, 10, 'r', min_speed=20, min_activation_speed=20)
       
    # Initialize cannon
    cannon = Cannon(5, 3, 7, 36, config)

    # Initialize LEDs
    last_time_a_button_was_pressed = -1
    front_led = LED('BOARD22')  # Use gpiozero numeration

    # Main loop
    try:
        # Start processes
        capture.start()
        show.start()

        # Initialize pygame
        pygame.init()
        pygame.mixer.init()

        # Play music
        init_music()
        pygame.mixer.music.play()
        music_paused = False

        # Initialize sounds
        exit_sound = init_one_sound(exit_sound_path)
        starwars_sounds = init_sounds(sound_paths_starwars)

        sounds_aim = init_sounds(sound_paths_aim)
        sounds_shoot = init_sounds(sound_paths_shoot)

        current_sound_mode = 'starcraft'

        # Wait for a controller
        control_id = wait_for_a_controller(function=front_led.toggle)
        front_led.on()
        
        # Initialize controller
        delay_between_button_inputs = 0.25

        controller = control_init(control_id)

        while 1:
            # Read the inputs from the controller
            control_values = get_button(joystick_deadzone=0.08)

            # DC motors
            m1.move(-control_values['axis1'], control_values['axis0'])
            m2.move(-control_values['axis1'], control_values['axis0'])

            # The turret
            up_or_down_value = (control_values['axis4']+1)/2 - (control_values['axis5']+1)/2
            servo.rotate_by(up_or_down_value, do_not_use_sleep=True)  # Up/Down
            stepper.rotate(control_values['axis2'], turn_coefficient=50, delay=0.002)  # Left/Right

            # Update the turret angle
            shared_turret_angle.value = stepper.turret_angle

            # LEDs
            if control_values['D-pad-Y'] == 1:
                if time() - last_time_a_button_was_pressed > delay_between_button_inputs:
                    front_led.toggle()
                last_time_a_button_was_pressed = time()

            # Stop and play music
            if control_values['D-pad-X'] == -1:
                if time() - last_time_a_button_was_pressed > delay_between_button_inputs:
                    music_paused = toggle_music(music_paused)
                last_time_a_button_was_pressed = time()
            
            # Switch between StarWars shooting sounds and StarCraft
            if control_values['D-pad-X'] == 1:
                if time() - last_time_a_button_was_pressed > delay_between_button_inputs:
                    if current_sound_mode == 'starcraft':
                        play_random_sound(starwars_sounds, channel_id=0)
                        current_sound_mode = 'starwars'
                    elif current_sound_mode == 'starwars':
                        play_sound(sounds_shoot['orders sir'], channel_id=0)
                        current_sound_mode = 'starcraft'
                last_time_a_button_was_pressed = time()
            
            # Stop any sound playing
            if control_values['D-pad-Y'] == -1:
                if time() - last_time_a_button_was_pressed > delay_between_button_inputs:
                    stop_sound_on_channel(channel_id=0)
                last_time_a_button_was_pressed = time()
            
            # Play random sound
            if control_values['b']:
                if time() - last_time_a_button_was_pressed > delay_between_button_inputs:
                    play_random_sound(starwars_sounds, channel_id=0)

                last_time_a_button_was_pressed = time()
            
            # Shooting
            if control_values['x']:
                if cannon.check_command():
                    play_sound(sounds_shoot['yes sir'], channel_id=0, wait=True)
                    sleep(0.5)  # To ensure nothing else is moving
                    play_sound(sounds_shoot['tank setting up'], channel_id=0, wait=True)
                    cannon.shoot()

                    if current_sound_mode == 'starcraft':
                        play_random_sound_from_selection(sounds_shoot, channel_id=0, wait=True, selection=['tta-ta-ta-da', 'justice'])
                    elif current_sound_mode == 'starwars':
                        play_random_sound(starwars_sounds, channel_id=0)
            else:
                cannon.last_time_check = -1
            
            # Aiming
            if control_values['lb']:
                # If LB or Space is pressed
                play_sound(sounds_aim['identify target'], channel_id=0, wait=True)

                img = read_image_from_shared_memory(image_resol, n=1)
                success = start_aiming(images_array=img)

                if success:
                    play_sound(sounds_aim['nuclear missle ready'], channel_id=0, wait=True)
                else:
                    play_sound(sounds_aim['not enough energy'], channel_id=0, wait=True)
                    
            
            if closed.is_set():
                print('Closed via Q-button')
                play_sound(exit_sound, 0, True)
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
