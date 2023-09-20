import cv2
import RPi.GPIO as gp
from camera import initialize as camera_init
from motors import Servo, Stepper
from turret.cannon import Cannon
from configparser import ConfigParser
from turret.aim import aim
from camera.qrcode import get_qrcode_pyzbar
from math_part import find_middle, translate_origin_to_cannon, translate_image_point, sin, cos, find_angle_with_drag, \
    radians, pi, atan, degrees, translate_point_to_vertical_plane
from turret.distance_qr import distance
from time import sleep
from CameraCalibration.undistort import *


def start_aiming(image, detect):
    cv2.imwrite('crap0.jpg', image)
    image = undistort(image, cmx, dist)

    # Mirror the image, because the camera actually sees an inverted image
    image = cv2.flip(image, 0)
    print(image.size)

    qr = list(get_qrcode_pyzbar(image))
    if qr:
        coords = qr[0][0]  # Get the coordinates

        # Translate the "image" coordinates to "real" coordinates (with center of the camera as origin)
        real_points = [translate_image_point(point, current_resolution=(image.shape[1], image.shape[0])) for point in
                       coords]

        # Project the real coordinates on the vertical plane
        real_points = [
            translate_point_to_vertical_plane(point, angle_camera_x, angle_camera_y, focus_length, return_two=True) for
            point in real_points]
        print(real_points)

        # Draw borders around the QR-code and show the frame in separate window
        middle = find_middle(coords, give_int=True)
        print(middle, 'ff')
        image = cv2.polylines(image, [coords], 1, (0, 255, 0), 2)
        cv2.circle(image, middle, 2, (0, 0, 255), -1)
        cv2.circle(image, (image.shape[1] // 2, image.shape[0] // 2), 3, (255, 0, 0), -1)

        # Show the image with the target
        cv2.imshow('Target', cv2.flip(image, 0))
        cv2.circle(image, tuple(coords[0]), 5, (255, 0, 0), -1)
        cv2.circle(image, tuple(coords[1]), 5, (0, 0, 255), -1)
        cv2.circle(image, tuple(coords[2]), 5, (0, 255, 255), -1)
        cv2.imwrite('crap.jpg', image)

        ###########################

        # Calculate the distance from camera
        d_camera, M_coords = distance(real_points, focus=focus_length, qrcode_length=qr_width, image_size=image_size,
                                      return_middle=True)
        print('M:', M_coords)

        # Print it out
        print('Distance camera:', d_camera)

        # Calculate the distance and the angles from the center of the turret, only horizontal angle needed
        angle_turret_x, angle_turret_y, d_turret = translate_origin_to_cannon(d_camera, M_coords,
                                                                              coordinates_of_camera_with_respect_to_the_turret)

        # Calculate the distance and angles from the cannon
        angle_cannon_x, angle_cannon_y, d_cannon = translate_origin_to_cannon(d_camera, M_coords,
                                                                              coordinates_of_camera_with_respect_to_the_cannon)

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
    mass = 0.12 * 10 ** (-3)
    initial_velocity = float(config['constants']['initial velocity'])
    coefficient = 1 / 2 * 1.225 * 0.47 * pi * (0.006 / 2) ** 2
    g = 9.8122

    # Camera-related constants
    coordinates_of_camera_with_respect_to_the_turret = (
    0.035, -0.03, 0.04)  # Center of the turret, needed for correct horizontal angle
    coordinates_of_camera_with_respect_to_the_cannon = (
    0.034, -0.1, 0.045)  # Cannon itself, needed for correct vertical angle

    angle_camera_x = 0
    angle_camera_y = 0

    focus_length = 3.04 * 10 ** (-3)
    qr_width = 0.122
    image_size = (640, 480)

    cmx, dist = get_calib_params('CameraCalibration/calibration.pkl')

    # Initialize camera
    cap = camera_init(image_size[0], image_size[1])  # Camera initialization
    detect = cv2.QRCodeDetector()  # Set up detection

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
                      initial_step=initial_step, upper_border=120, lower_border=-120, config=config,
                      config_file=config_file)

    # Initialize cannon
    cannon = Cannon(5, 3, 7, 36, config)

    # Main loop
    try:
        while 1:
            success, img = cap.read()  # Read frame
            if not success:
                print("Failed to get a frame...")
                break

            # img = cv2.rotate(img, cv2.ROTATE_180)  # ROTATES THE IMAGE \\ (UN)COMMENT IF NEEDED

            cv2.imshow('Camera', img)

            k = cv2.waitKey(1)  # Read keyboard and wait

            if k % 256 == 32:
                # Space pressed
                start_aiming(img, detect=detect)

            if k % 256 == ord('f'):
                # F is pressed
                print('Prepairing to shoot...')
                sleep(0.5)

                cannon.shoot()

                print('--> Done')

            if k % 256 == ord('q'):
                # Q is pressed --> closing...
                raise KeyboardInterrupt

    except KeyboardInterrupt:
        print("Program closed...")
    finally:
        servo.kill()
        gp.cleanup()
        cap.release()
        cv2.destroyAllWindows()
        print("\nGoodbye!")
