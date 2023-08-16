import cv2 as cv
import numpy as np
from math_part import find_middle
from recognize_circles import rescale
from pyzbar.pyzbar import decode


def sort_numpy_array(array):  # Thanks, ChatGPT
    # Find the point closest to the top-left corner (point with the minimum sum of x and y coordinates)
    start_point_index = np.argmin(np.sum(array, axis=1))

    # Reorder the points array so that the start point comes first
    array = np.roll(array, -start_point_index, axis=0)

    return array


def get_qrcode_cv(img, target=None, detect=None):
    if detect is None:
        detect = cv.QRCodeDetector()
    
    value, decoded_info, points, straight_qrcode = detect.detectAndDecodeMulti(img)

    if value:  # If found any; value is either True or False
        for string, coords in zip(decoded_info, points):
            if target is not None:
                if string != target:
                    continue
            yield coords.astype(int), string
    else:
        return


def get_qrcode_pyzbar(img, target=None):
    for d in decode(img):
        string = d.data.decode()
        if target is not None:
            if string != target:
                continue
        
        #array = np.array(d.polygon)  # Translate the array to numpy array
        array = np.flip(np.array(d.polygon), axis = 0)  # Translate the array to numpy array and reverse the order of the points, 
                                                        # so that it has the same form as with Open CV

        array = sort_numpy_array(array)

        yield array, string


def get_average_coordinates_from_array_of_images(array, target=None):
    qr_codes = []

    for img in array:
        qr = list(get_qrcode_pyzbar(img, target=target))
        
        if qr:
            qr_codes.append(qr[0][0])
    if not qr_codes:
        return [[]]
    qr_codes = np.array(qr_codes, dtype=np.int32)
    
    qr_codes_rounded = np.array([round(i) for i in np.mean(qr_codes, 0).flatten()], dtype=np.int32).reshape(qr_codes[0].shape)

    return qr_codes_rounded


if __name__ == '__main__':
    img = cv.imread('Camera Calibration/caliResult1.png')
    print(img.shape)

    # img = rescale(img, 1)

    detect = cv.QRCodeDetector()
    ex = 0
    qr = list(get_qrcode_pyzbar(img))
    if qr:
        for points, string in qr:
            print(points)
            cv.circle(img, tuple(points[0]), 5, (255, 0, 0), -1)
            cv.circle(img, tuple(points[1]), 5, (0, 0, 255), -1)
            cv.circle(img, tuple(points[2]), 5, (0, 255, 255), -1)
            middle = find_middle(points, give_int=True)
            print(middle)
            img = cv.polylines(img, [points], 1, (0, 255, 0), 2)
            cv.circle(img, middle, 2, (0, 0, 255), -1)
            cv.circle(img, (img.shape[1]//2, img.shape[0]//2), 3, (255, 0, 0), -1)

    else:
        print("Alas")

    cv.imshow('QR', img)
    cv.waitKey(0)