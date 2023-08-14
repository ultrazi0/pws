import multiprocessing as mp
import numpy as np
import socket
import os
from numpysocket import NumpySocket
import cv2

from camera import initialize




def child():
    cap = initialize()

    with NumpySocket(socket.AF_UNIX) as s:
        s.connect(socket_path)
        while(cap.isOpened()):
            ret, frame = cap.read()
            if ret is True:
                try:
                    s.sendall(frame)
                except:
                    break
            else:
                break


def parent():
    with NumpySocket(socket.AF_UNIX) as s:
        s.bind(socket_path)

        while True:
            try:
                s.listen()
                conn, addr = s.accept()
                while conn:
                    frame = conn.recv()
                    if len(frame) == 0:
                        break
                    
                    cv2.imshow('Frame', frame)
                    
                    # Press Q on keyboard to exit
                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        exit(1)
            except ConnectionResetError:
                pass

if __name__ == "__main__":
    socket_path = '/tmp/test_socket'

    pid = os.fork()

    if pid > 0:
        parent()
    elif pid == 0:
        child()
