import cv2
import numpy as np
from pyzbar.pyzbar import decode
from qrcode import get_qrcode_cv

camera_id = 0
delay = 1
window_name = 'OpenCV pyzbar'

cap = cv2.VideoCapture(camera_id)

detect = cv2.QRCodeDetector()

while True:
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    img = frame.copy()

    if ret:
        for d in decode(frame):
            s = d.data.decode()
            print(np.array(d.polygon))
            frame = cv2.polylines(frame, [np.array(d.polygon)], True, (255, 0, 0), 2)
            cv2.circle(frame, tuple(d.polygon[0]), 5, (0, 0, 255), -1)
            cv2.circle(frame, tuple(d.polygon[1]), 5, (0, 255, 0), -1)

        cv2.imshow(window_name, frame)

        qr = list(get_qrcode_cv(img, detect=detect))
        print(qr)
        if qr:
            for points, message in qr:
                cv2.polylines(img, [points], 1, (0, 255, 0), 2)

        cv2.imshow("Result", img)

    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break

cv2.destroyWindow(window_name)