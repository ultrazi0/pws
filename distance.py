from recog import getObjects
import cv2 as cv
import numpy as np
from recognize_circles import get_circles, rescale


def focal_length(known_width, width_in_image, known_distance):
    return (width_in_image * known_distance) / known_width

def distance(width_in_image, known_width, focal_length):
    return (known_width * focal_length) / width_in_image

def get_distance(img, known_width, known_distance, known_width_in_image, focus, draw=True):
    img, circles = get_circles(img, draw=draw, min_dist=300, min_radius=0)

    reworked_circles = None  # This might consume a bit too many resources, but for now it should work. If possible figure out a way to add a float to a ndarray
    if circles is not None:
        reworked_circles = circles.tolist()
        for i, (x, y, r) in enumerate(circles):
            d = distance(2*r, known_width, focus)
            reworked_circles[i].append(d)
            #print(circles[i], "dfdsf")
            #circles[i] = np.append(circles[i], d)
            cv.putText(img, str(round(d, 2))+"m", (x+20, y+30), cv.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    
    return img, reworked_circles



if __name__ == "__main__":
    known_width = 0.18  # in meters
    known_distance = 0.3  # in meters
    known_width_in_image = 372  # 1472  # in pixels

    cap = cv.VideoCapture(0)
    cap.set(2, 640)
    cap.set(3, 480)

    focus = focal_length(known_width, known_width_in_image, known_distance)

    try:
        while 1:
        
            success, img = cap.read()

            img, circles = get_distance(img, known_width, known_distance, known_width_in_image, focus)

            print(circles)
            cv.imshow('Output', img)
        
            cv.waitKey(1)
    except KeyboardInterrupt:
        cv.destroyAllWindows()