import RPi.GPIO as gp
from recog import initialize as camera_init, cv2
from servo import Servo
from stepper import Stepper
from canon import Canon
from configparser import ConfigParser
from aim import aim
from qrcode import get_qrcode_pyzbar
from math_part import find_middle, translate_origin_to_canon, translate_image_point, sin, cos, find_angle_with_drag, radians, pi, atan, degrees
from distance_qr import distance
from time import sleep
from CameraCalibration.undistort import *


def start_aiming(image, detect):
    cv2.imwrite('crap0.jpg', image)
    image = undistort(image, cmx, dist)

    # Rotate the image, because the camera actually sees an inverted image
    image = cv2.flip(image, 0)
    print(image.size)

    qr = list(get_qrcode_pyzbar(image))
    if qr:
        coords = qr[0][0]  # Get the coordinates

        # Translate the "image" coordinates to "real" coordinates (with center of the camera as origin)
        real_points = [translate_image_point(point, current_resolution=(image.shape[1], image.shape[0])) for point in coords]
        print(real_points)

        # Draw borders around the QR-code and show the frame in separate window
        middle = find_middle(coords, give_int=True)
        print(middle, 'ff')
        image = cv2.polylines(image, [coords], 1, (0, 255, 0), 2)
        cv2.circle(image, middle, 2, (0, 0, 255), -1)
        cv2.circle(image, (image.shape[1]//2, image.shape[0]//2), 3, (255, 0, 0), -1)
        
        # Show the image with the target
        cv2.imshow('Target', cv2.flip(image, 0))
        cv2.circle(image, tuple(coords[0]), 5, (255, 0, 0), -1)
        cv2.circle(image, tuple(coords[1]), 5, (0, 0, 255), -1)
        cv2.circle(image, tuple(coords[2]), 5, (0, 255, 255), -1)
        cv2.imwrite('crap.jpg', image)

        ###########################

        # Calculate the distance from camera
        d_camera, M_coords = distance(real_points, focus=focus_length, qrcode_length=qr_width, image_size=image_size, return_middle=True)
        print('M:', M_coords)

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
        print('Alas, nothing, Sir!')
    
    print("We're done, Sir!")


if __name__ == '__main__':
    # Initialize config
    config = ConfigParser()
    config_file = 'cfg.ini'
    config.read(config_file)

    gp.setmode(gp.BOARD)  # Set the pin-numbering

    # Projectile-related constants
    mass = 0.12*10**(-3)
    initial_velocity = float(config['constants']['initial velocity'])
    coefficient = 1/2*1.225*0.47*pi*(0.006/2)**2
    g = 9.8122

    # Camera-related constants
    coordinates_of_camera_with_respect_to_the_turret = (0.03, -0.03, 0.03)  # Center of the turret, needed for correct horizontal angle
    coordinates_of_camera_with_respect_to_the_canon = (0.045, -0.09, 0.033)  # Canon itself, needed for correct vertical angle

    focus_length = 3.04*10**(-3)
    qr_width = 0.122
    image_size = (640, 480)

    # Initialize camera
    cap = camera_init(image_size[0], image_size[1])  # Camera initialization
    detect = cv2.QRCodeDetector()  # Set up detection

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
    
    # Initialize canon
    canon = Canon(5, 3, 7, 36, config)

    # Main loop
    cmx, dist = get_calib_params('CameraCalibration/calibration.pkl')
    try:
        while 1:
            success, img = cap.read()  # Read frame
            if not success:
                print("Failed to get a frame...")
                break

            #img = cv2.rotate(img, cv2.ROTATE_180)  # ROTATES THE IMAGE \\ (UN)COMMENT IF NEEDED
            
            cv2.imshow('Camera', img)

            k = cv2.waitKey(1)  # Read keyboard and wait

            if k%256 == 32: 
                # Space pressed
                start_aiming(img, detect=detect)
            
            if k%256 == ord('f'):
                # F is pressed
                print('Prepairing to shoot...')
                sleep(0.5)

                canon.shoot()

                print('--> Done')
            
            if k%256 == ord('q'):
                # Q is pressed --> closing...
                raise KeyboardInterrupt

    except KeyboardInterrupt:
        print("Programm closed...")
    finally:
        servo.kill()
        gp.cleanup()
        cap.release()
        cv2.destroyAllWindows()
        print("\nGoodbye!")
