import cv2 as cv
import numpy as np


def rescale(img, scale):
    width = int(img.shape[1] * scale)
    height = int(img.shape[0] * scale)

    return cv.resize(img, (width, height), interpolation=cv.INTER_AREA)


def get_circles(image, draw=True, dp=1.2, min_dist=100, min_radius=0, max_radius=0):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, dp, minDist=min_dist, minRadius=min_radius, maxRadius=max_radius)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")

        if draw:
            for (x, y, r) in circles:
                cv.circle(image, (x, y), r, (0, 255, 0), 4)
                cv.rectangle(image, (x-3, y-3), (x+3, y+3), (255, 0, 0), 1)
    
    return image, circles

if __name__ == '__main__':
    img = cv.imread('target.jpg')
    output = rescale(img.copy(), 0.25)
    output, circles = get_circles(output, min_dist=75)

    print(circles)
    cv.imshow('Output', output)
    cv.waitKey(0)

    cv.destroyAllWindows()