import numpy as np
import cv2 as cv
from time import sleep, time
from multiprocessing import Process, Event
from multiprocessing.shared_memory import SharedMemory
from camera import initialize


def sender(shape, event):
    mem = SharedMemory('Mem')

    cap = initialize()

    print('Sender: Running', flush=True)
    while not event.is_set():
        success, img = cap.read()
        array = np.ndarray(img.shape, dtype=img.dtype, buffer=mem.buf)

        array[:] = img[:]

        print(f'Sender: sending\n{array}', flush=True)

        cv.waitKey(1)

    print('Sender: Done', flush=True)
    mem.close()


def receiver(shape, event):
    mem = SharedMemory('Mem')

    print('Receiver: Running', flush=True)
    
    try:
        while 1:

            item = np.ndarray(shape=shape, buffer=mem.buf, dtype=np.uint8)
            print(f'>receiver got:\n{item}', flush=True)

            cv.imshow('QR', item)
            k = cv.waitKey(1)

            if k%256 == ord('q'):
                raise KeyboardInterrupt
    except KeyboardInterrupt:
        print('Closing...')
        event.set()

    print('Receiver: Done')
    mem.close()


if __name__ == '__main__':
    shm = SharedMemory(create=True, size=2**20, name='Mem')
    closed = Event()

    shape = (480, 640, 3)

    sender_process = Process(target=sender, args=(shape, closed))
    receiver_process = Process(target=receiver, args=(shape, closed))

    sender_process.start()
    receiver_process.start()

    sender_process.join()
    receiver_process.join()

    shm.close()
    shm.unlink()
