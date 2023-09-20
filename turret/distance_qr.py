from math_part import *


def distance(coordinates, focus, qrcode_length, image_size, origin=None, return_middle=False):
    middle = (image_size[0] // 2, image_size[1] // 2)

    # Points in two dimensions

    # print("***WARNING! TEST MODE ACTIVATED***")
    # I_2 = (coordinates[0][0]-middle[0], coordinates[0][1]-middle[1])
    # J_2 = (coordinates[1][0]-middle[0], coordinates[1][1]-middle[1])
    # K_2 = (coordinates[2][0]-middle[0], coordinates[2][1]-middle[1])
    # L_2 = (coordinates[3][0]-middle[0], coordinates[3][1]-middle[1])

    I_2, J_2, K_2, L_2 = coordinates

    # Absolute coordinates of the points --> in three dimensions
    I = (I_2[0], -focus, I_2[1])
    J = (J_2[0], -focus, J_2[1])
    K = (K_2[0], -focus, K_2[1])
    L = (L_2[0], -focus, L_2[1])

    # Absolute coordinates of origin
    O = origin if origin is not None else (0, 0, 0)

    # Finding middle of the image (point Q)
    LJ = get_line(L_2, J_2)
    IK = get_line(I_2, K_2)

    Q_2 = find_intersect_in_plane(LJ, IK)
    Q = (Q_2[0], -focus, Q_2[1])

    ################################################
    # Finding angle at which the QR-code is turned #
    ################################################

    # Extend the sides until intersection
    LK = get_line(L_2, K_2)
    IJ = get_line(I_2, J_2)

    # Point of intersection
    H1_2 = find_intersect_in_plane(LK, IJ)
    if H1_2 is not False:
        # Lines are not parallel
        H1 = (H1_2[0], -focus, H1_2[1])

        # Point that makes right triangle with origin (O) and the point of intersection (H1), it lies on the line QH1
        J1 = find_intersect_in_space(Q, get_vector(Q, H1), O, perpendicular_vector(Q, H1, O))

        # Find the angle
        angle = find_angle(H1, J1, O)

        if length_2d(L_2, I_2) > length_2d(K_2, J_2):
            angle *= -1

        # Find the points that correpond with the middle points of the vertical sides of the QR-code
        QH1 = get_line(Q_2, H1_2)  # Points lie on this line

        R_2, S_2 = find_middle_points([I_2, J_2, K_2, L_2], QH1)

        R = (R_2[0], -focus, R_2[1])

    else:
        # Lines are parallel --> QR-code is not rotated
        angle = 0

        QH1 = get_parallel_line(Q_2, LK[0])  # QH1||LK||IJ

        R_2, S_2 = find_middle_points([I_2, J_2, K_2, L_2], QH1)

        R = (R_2[0], -focus, R_2[1])

        # Point that makes right triangle with origin (O) and lies on the line QH1
        J1 = find_intersect_in_space(Q, get_vector(Q, R), O, perpendicular_vector(Q, R, O))

    S = (S_2[0], -focus, S_2[1])

    R_translated = get_translated_coordinates(R, O, J1)
    S_translated = get_translated_coordinates(S, O, J1)

    # Calculate the distance
    d = calculate_distance(S_translated[0], R_translated[0], R_translated[1], angle, qrcode_length, angle_in_degrees=False)

    if return_middle:
        return d, Q
    return d






if __name__ == "__main__":
    known_width = 0.122  # in meters
    known_distance = 0.403  # in meters
    known_width_in_image = 157  # in pixels

    coords = [[2.36671, 2.51463], 
              [1.48728, 2.52269], 
              [1.20068, 3.15011], 
              [2.51182, 3.15391]]
    f = 2
    img = [2*2, 3*2]
    l = 4

    d = distance(coords,f, l, img)

    print(d)
