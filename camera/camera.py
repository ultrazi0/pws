import cv2
import numpy as np
from time import sleep
from multiprocessing import Process, Event
from multiprocessing.shared_memory import SharedMemory
from os import getpid


def initialize(frame_width: int =640, frame_height: int =480, camera: int =0):
    cap = cv2.VideoCapture(camera)
    cap.set(3, frame_width)
    cap.set(4, frame_height)
    return cap


def edit_image(image, val=None):
    height, width = image.shape[:2]

    cv2.circle(image, (width//2, height//2), 3, (255, 0, 0), -1)

    if val is not None:
        turret_angle = round(val.value, 1)
        string = 'Turret angle: ' + str(turret_angle)

        image = cv2.putText(
            img = image,
            text = string,
            org = (width-250, 25),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 0.75,
            color = (125, 246, 55),
            thickness = 2
        )
    
    return image


def process_capture(resol, event_closed):
    print(f'Capture process ({getpid()}): Running', flush=True)

    # Shared memory for sharing the image
    mem = SharedMemory('ImageMemory')

    cap = initialize(resol[0], resol[1])

    try:
        while not event_closed.is_set():
            # Read the image
            success, img = cap.read()
            if not success:
                print('Failed to get a frame...')
                break

            # Create an array with the same properties as the image
            array = np.ndarray(img.shape, dtype=img.dtype, buffer=mem.buf)

            # Copy the image into the array
            array[:] = img[:]

            cv2.waitKey(1)
    except KeyboardInterrupt:
        print('Capture: Closing', flush=True)
    finally:
        mem.close()
        cap.release()
        print('Sender: Done', flush=True)


def process_show(resol, event_closed, config=None):
    print(f'Receiver process ({getpid()}): Running', flush=True)

    image_channels = 3

    # Shared memory to read image
    mem = SharedMemory('ImageMemory')

    try:
        while 1:
            # Read image
            img = np.ndarray(shape=(resol[1], resol[0], image_channels), buffer=mem.buf, dtype=np.uint8)

            # Show image
            img = edit_image(img, config)
            cv2.imshow('Camera', img)
            k = cv2.waitKey(1)

            if k%256 == ord('q'):
                raise KeyboardInterrupt
            
            if event_closed.is_set():
                print('Receiver: Heard that must be closed...', flush=True)
                break
    except KeyboardInterrupt:
        print('Receier: Closing...', flush=True)
    finally:
        event_closed.set()
        
        mem.close()

        cv2.destroyAllWindows()
        print('Receiver: Done', flush=True)


def read_image_from_shared_memory(resol, n=1, name='ImageMemory', delay=1):
    image_channels = 3

    # Set up shared memory
    mem = SharedMemory(name)

    # Create an empty array for future images
    images = np.empty((n, resol[1], resol[0], 3), dtype=np.uint8)

    for i in range(n):

        # Read the image from memory
        img = np.ndarray(shape=(resol[1], resol[0], image_channels), buffer=mem.buf, dtype=np.uint8)

        # Write it into the array
        images[i] = img
        
        # Wait for a new image/frame to come up
        sleep(delay)
    
    # Close the memory
    mem.close()
    
    return images


if __name__ == "__main__":
    from math_part import find_middle
    from camera.qrcode import get_qrcode_pyzbar

    known_width = 0.18  # in meters
    known_distance = 0.403  # in meters
    known_width_in_image = 157  # in pixels

    camera_resol = (640, 480)

    shm = SharedMemory(name='ImageMemory', create=True, size=2**20)
    closed = Event()

    capture = Process(target=process_capture, args=(camera_resol, closed))
    show = Process(target=process_show, args=(camera_resol, closed))

    try:
        capture.start()
        show.start()
        # sleep(5)

        # images = read_image_from_shared_memory(camera_resol, 5)

        # for i, img in enumerate(images):
        #     print('------'+str(i)+'------')
        #     print(img)
        #     qr = list(get_qrcode_pyzbar(img))
        #     if qr:
        #         for points, message in qr:
        #             middle = find_middle(points, give_int=True)
        #             cv2.polylines(img, [points], 1, (0, 255, 0), 2)       
        #             cv2.circle(img, middle, 4, (0, 0, 255), -1)
    
        #     cv2.imshow(str(i), img)
        # while 1:
        #     if cv2.waitKey(1) % 0xFF == ord("q"):
        #         break

        capture.join()
        show.join()
    finally:
        shm.close()
        shm.unlink()
        print('THE END')
