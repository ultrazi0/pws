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


def start_aiming(image, detect):

    image = cv2.rotate(image, cv2.ROTATE_180)

    qr = list(get_qrcode_pyzbar(image))
    if qr:
        coords = qr[0][0]  # Get the coordinates

        # Translate the "image" coordinates to "real" coordinates (with center of the camera as origint)
        real_points = [translate_image_point(point) for point in coords]

        # Draw borders around the QR-code and show the frame in separate window
        middle = find_middle(coords, give_int=True)
        print(middle, 'ff')
        image = cv2.polylines(image, [coords], 1, (0, 255, 0), 2)
        cv2.circle(image, middle, 2, (0, 0, 255), -1)
        cv2.circle(image, (320, 240), 3, (255, 0, 0), -1)

        ###########################

        # Calculate the distance from camera
        d_camera, M_coords = distance(real_points, focus=focus_length, qrcode_length=qr_width, image_size=image_size, return_middle=True)

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

        # Print everything out
        print("Distance canon", d_canon)
        print('x-angle:', angle_x)
        print('y-angle:', angle_canon_y)



    else:
        print('Alas, nothing, Sir!')
    
    print("We're done, Sir!")


if __name__ == '__main__':
    img = cv2.imread('crap0.jpg')
    detect = cv2.QRCodeDetector()

    # Camera-related constants
    coordinates_of_camera_with_respect_to_the_turret = (0.03, 0, 0.03)  # Center of the turret, needed for correct horizontal angle
    coordinates_of_camera_with_respect_to_the_canon = (0.04, 0, 0)  # Canon itself, needed for correct vertical angle

    focus_length = 3.04*10**(-3)
    qr_width = 0.122
    image_size = (640, 480)


    start_aiming(img, detect=detect)
