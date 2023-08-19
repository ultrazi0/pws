import pickle
import cv2 as cv
import sys

sys.path.append('/home/pi/pws')
from qrcode import get_qrcode_pyzbar, find_middle

with open('calibration.pkl', 'rb') as cal_file:
    cmx, dist = pickle.load(cal_file)

print(cmx)
print()
print(dist)

img = cv.imread('images/crap4.jpg')
h, w = img.shape[:2]
newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cmx, dist, (w,h), 1, (w,h))



# Undistort
dst = cv.undistort(img, cmx, dist, None, newCameraMatrix)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w].copy()
print('shape:', dst.shape)

qr = list(get_qrcode_pyzbar(dst))
if qr:
    for points, string in qr:
        print(points)
        cv.circle(dst, tuple(points[0]), 5, (255, 0, 0), -1)
        cv.circle(dst, tuple(points[1]), 5, (0, 0, 255), -1)
        cv.circle(dst, tuple(points[2]), 5, (0, 255, 255), -1)
        middle = find_middle(points, give_int=True)
        print(middle)
        dst = cv.polylines(dst, [points], 1, (0, 255, 0), 2)
        cv.circle(dst, middle, 4, (0, 0, 255), -1)
        cv.circle(dst, (round(cmx[0][2]), round(cmx[1][2])), 3, (255, 0, 0), -1)




# cv.circle(dst, (316, 248), 4, (255,0,0), -1)
# cv.circle(dst, (214, 149), 4, (0,0,255), -1)
cv.circle(dst, (dst.shape[1]//2, dst.shape[0]//2), 4, (0,255,0), -1)

# cv.circle(dst1, (316, 248), 4, (255,0,0), -1)
# cv.circle(dst1, (214, 149), 4, (0,0,255), -1)
# cv.circle(dst1, (dst1.shape[1]//2, dst1.shape[0]//2), 4, (0,255,0), -1)

cv.imshow('img', dst)
#cv.imshow('img1', dst1)
while 1:
    k = cv.waitKey(1)
    if k%256 == ord('q'):
        break