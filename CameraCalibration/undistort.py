import pickle
import cv2 as cv

def undistort(img, calibration_matrix, dist):
    h, w = img.shape[:2]
    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(calibration_matrix, dist, (w,h), 1, (w,h))

    # Undistort
    dst = cv.undistort(img, calibration_matrix, dist, None, newCameraMatrix)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]

    return dst


def get_calib_params(filepath):
    with open(filepath, 'rb') as cal_file:
        cmx, dist = pickle.load(cal_file)
    
    return cmx, dist


if __name__ == "__main__":
    img = cv.imread('images/crap1.jpg')

    cmx, dist = get_calib_params('calibration.pkl')

    dst = undistort(img, cmx, dist)

    cv.imshow('img', img)
    cv.imshow('dst', dst)

    while 1:
        k = cv.waitKey(1)
        if k%256 == ord('q'):
            break

