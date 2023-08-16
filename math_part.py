import numpy as np
import sympy as s
from configparser import ConfigParser
from math import *


def average(array):
    return sum(array)/len(array)


def set_angle_between_borders(angle):
    """Sets the angle between -180 and 180"""
    amount_of_semicircles = angle // 180
    return angle - 180 * (amount_of_semicircles+amount_of_semicircles%2)


def angle_turret(turret_angle, number_of_teeth_stepper=None, number_of_teeth_turret=None, config=None):
    """
    Caclulates the stepper angle needed to turn the turret by turret_angle
    
    ---Numbers of teeth can be either directly specified or read from the config file---
    """
    if config is not None:
        number_of_teeth_stepper = int(config['constants']['number of teeth stepper'])
        number_of_teeth_turret = int(config['constants']['number of teeth turret'])
    else:
        if (number_of_teeth_turret is None) or (number_of_teeth_stepper is None):
            raise ValueError('If config is not provided number_of_teeth_stepper and number_of_teeth_turret must both be provided')
    
    return turret_angle * number_of_teeth_turret / number_of_teeth_stepper


def angle_canon(canon_angle, dx, dy, r, give_degrees=True, **kwargs):
    canon_angle = radians(canon_angle)

    ab = kwargs['ab'] if 'ab' in kwargs else sqrt(dx**2 + dy**2)
    phi = kwargs['phi'] if 'phi' in kwargs else atan(dy/dx)
    
    max_angle_beta = asin(r/ab) + phi
    if canon_angle > max_angle_beta:
        print('Sorry, Sir! The turret cannot go that high.\n' +
              '    We will still lift it as high as possible though')
        angle_alpha = pi + phi + asin(dy/r) - acos(r/ab)
    else:
        theta = asin(sin(canon_angle-phi)*ab/r)
        angle_alpha = theta + canon_angle + asin(dy/r)

    if give_degrees:
        return degrees(angle_alpha)
    return angle_alpha


def calculate_angle_without_drag(x, y, v0, x0=0, y0=0, g=9.81, give_degrees=True, return_both=False):
    root = sqrt(v0**4-g**2*(x-x0)**2-2*g*v0**2*(y-y0))
    angle1 = atan((v0**2-root)/(g*(x-x0)))
    angle2 = atan((v0**2+root)/(g*(x-x0)))

    
    if give_degrees:
        if not return_both:
            return degrees(angle1)
        return degrees(angle1), degrees(angle2)
    if not return_both:
        return angle1
    return angle1, angle2


def find_middle(coordinates, give_int=False):
    if len(coordinates) != 4:
        raise ValueError('Too many or not enough coordinates')
    
    x1, y1 = coordinates[0]
    x2, y2 = coordinates[1]
    x3, y3 = coordinates[2]
    x4, y4 = coordinates[3]

    # If lines are parallel
    if (x3-x1==0) and (x4-x2==0) and (x1!=x2):
        raise ValueError('Lines appear to be parralel (x=a)')
    elif (y3-y1==0) and (y4-y2==0) and (y1!=y2):
        raise ValueError('Lines appear to be parralel (y=a)')
    
    # If one of the lines is parralel with Oy-axis
    elif (x3-x1==0) and (x4-x2!=0):
        x = x1
        y = y2 + (x1-x2)/(x4-x2)*(y4-y2)
    elif (x4-x2==0) and (x3-x1!=0):
        x = x2
        y = y1 + (x2-x1)/(x3-x1)*(y3-y1)
    
    # If one of the lines is parralel with Ox-axis
    elif (y3-y1==0) and (y4-y2!=0):
        y = y1
        x = x2 + (y1-y2)/(y4-y2)*(x4-x2)
    elif (y4-y2==0) and (y3-y1!=0):
        y = y2
        x = x1 + (y2-y1)/(y3-y1)*(x3-x1)
    
    # In all other cases
    else:
        # Division by zero when the opposite points have the same x- or y-coordinate
        x = (y2-x2*(y4-y2)/(x4-x2)-(y1-x1*(y3-y1)/(x3-x1)))/((y3-y1)/(x3-x1)-(y4-y2)/(x4-x2))
        y = y1 + (x-x1)*(y3-y1)/(x3-x1)

    if give_int:
        return (round(x), round(y))
    return (x, y)


def length_2d(coords1, coords2):
    x1, y1 = coords1
    x2, y2 = coords2

    return sqrt((x1-x2)**2+(y1-y2)**2)


def length_3d(coords1, coords2):
    x1, y1, z1 = coords1
    x2, y2, z2 = coords2

    return sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)


def find_focus_length(distance, width, width_in_image):
    return distance * (width_in_image/width)


def find_angle_x(coordinates, image_middle, known_distance, known_width, known_width_in_image):
    focus = find_focus_length(known_distance, known_width, known_width_in_image)

    middle = find_middle(coordinates)

    angle_x = degrees(atan((middle[0]-image_middle[0])/focus))

    return angle_x


def find_angle_y_laser(coordinates, image_middle, known_distance, known_width, known_width_in_image):
    focus = find_focus_length(known_distance, known_width, known_width_in_image)

    middle = find_middle(coordinates)

    angle_y = degrees(atan((middle[1]-image_middle[1])/focus))

    return angle_y


def distance(coordinates, known_width_in_image, known_distance, return_middle=False):  # DOES NOT WORK -- there is a new one
    # For clarity refer to Geogebra "perspective"
    
    x1, y1 = coordinates[0]  # point H
    x2, y2 = coordinates[1]  # point G
    x3, y3 = coordinates[2]  # point F
    x4, y4 = coordinates[3]  # point E

    GH = length_2d((x3,y3), (x2, y2))
    
    x_mid, y_mid = find_middle(coordinates)  # point I

    x_J, y_J = find_middle([
        [x1, y1],
        [x_mid, y_mid],
        [x4, y4],
        [x_mid-GH, y_mid]  # FIXME: only when not turned
    ])
    
    x_K, y_K = find_middle([
        [x2, y2],
        [x_mid, y_mid],
        [x3, y3],
        [x_mid+GH, y_mid]  # FIXME: only when not turned
    ])

    JK = length_2d((x_J, y_J), (x_K, y_K))
    #print(x_mid, y_mid)
    #print(coordinates, (x_mid, y_mid), (x_J, y_J), (x_K, y_K))

    if return_middle:
        return known_distance * (known_width_in_image/JK), [x_mid, y_mid]
    return known_distance * (known_width_in_image/JK)  # FIXME: does not take offset into account


# Distance two-dimensional space
def get_line(point1, point2):
    k = (point2[1]-point1[1])/(point2[0]-point1[0])
    b = point1[1] - k * point1[0]

    return k, b


def get_parallel_line(point, k):
    b = point[1] - k * point[0]

    return k, b


def vertical_line(point1, point2=None):
    if point2 is not None:
        if point1[0] != point2[0]:
            raise ValueError('Wrong fuction used: x-coordinates of the points are not the same --> use the normal function')
    
    y = point1[0]

    return 'infinity', y


def find_intersect_in_plane(line1, line2):
    """
    Returns point of intersection of two lines in two-dimensional space

    :param line1: (k, b) of the first line
    :param line2: (k, b) of the second line
    :return: coordinates of the point of intersection
    """

    if line1[0] == line2[0]:
        # Lines are parallel
        return False
    
    if (line1[0] == 'infinity') or (line2[0] == 'infinity'):
        # One of the lines is vertical 
        if line1[0] == 'infinity':
            vert_line = line1
            normal_line = line2
        else:
            vert_line = line2
            normal_line = line1
        
        x = vert_line[1]
        y = normal_line[0] * x + normal_line[1]
        
        return x, y

    x = (line1[1]-line2[1])/(line2[0]-line1[0])
    y = line1[0] * x + line1[1]

    return x, y


def find_middle_points(coordinates, line):
    """
    Returns two points that correpond with the middle points of the vertical sides of the QR-code

    :param coordinates: Coordinates of the corners of the image
    :param line: Line that goes through the middle and on which the points lie
    :return: Two coordinates of two points in two-dimentional form
    """
    A_2 = coordinates[0]  # point I_2
    B_2 = coordinates[1]  # point J_2
    C_2 = coordinates[2]  # point K_2
    D_2 = coordinates[3]  # point L_2

    if (A_2[0] == D_2[0]) and (B_2[0] == C_2[0]):
        # Both "vertical" sides are really vertical
        AD = vertical_line(D_2, A_2)
        BC = vertical_line(C_2, B_2)

    elif D_2[0] == A_2[0]:
        # LI is vertical
        AD = vertical_line(D_2, A_2)
        BC = get_line(C_2, B_2)

    elif C_2[0] == B_2[0]:
        # KJ is vertical
        AD = get_line(D_2, A_2)
        BC = vertical_line(C_2, B_2)
    else:
        AD = get_line(A_2, D_2)
        BC = get_line(B_2, C_2)
    
    R_2 = find_intersect_in_plane(BC, line)
    S_2 = find_intersect_in_plane(AD, line)

    return R_2, S_2


# Disnance - three-dimensional space
def get_vector(start, end):
    return end[0] - start[0], end[1] - start[1], end[2] - start[2]


def perpendicular_vector(point1, point2, origin):
    """
    Gives vector perpendicular to line through point1 and point2 that goes through the origin

    :param point1: Coordinates of the first point in space
    :param point2: Coordinates of the second point in space
    :param origin: Absolute coordinates of the origin in space
    :return: Vector in form of a point
    """
    r = get_vector(point1, point2)  # directional vector for the line between A and B

    t = (r[0]*(origin[0]-point1[0])+r[1]*(origin[1]-point1[1])+r[2]*(origin[2]-point1[2]))/(r[0]**2+r[1]**2+r[2]**2)

    n = (-origin[0]+point1[0]+r[0]*t, -origin[1]+point1[1]+r[1]*t,
         -origin[2]+point1[2]+r[2]*t)  # directional vector for perpendicular line

    return n


def find_intersect_in_space(base1, r1, base2, r2):
    """
    Finds point of intersection of two lines.

    :param base1: base vector of the first line
    :param r1: directional vector of the first line
    :param base2: base vector of the second line
    :param r2: directional vector of the second line
    :return: point of intersection
    """
    # Assume y-coordinate is 0
    if r1[1] != 0:
        raise ValueError('Assumption failed - cannot solve with y-coordinate not 0')

    t = (base1[1]-base2[1])/r2[1]

    return base2[0]+r2[0]*t, base2[1]+r2[1]*t, base2[2]+r2[2]*t


def find_angle(point, point90, origin, return_degrees=False):
    ang = asin(
        sqrt((point90[0]-origin[0])**2+(point90[1]-origin[1])**2+(point90[2]-origin[2])**2) /
        sqrt((point[0]-origin[0])**2+(point[1]-origin[1])**2+(point[2]-origin[2])**2)
    )
    if return_degrees:
        return degrees(ang)
    return ang


def get_translated_coordinates(point, origin, config_point):
    """
    Translates point to a two-dimensional system of coordinates that goes through "point", "origin" and "config_point"

    :param point: absolute coordinates of the point that needs to be translated
    :param origin: absolute coordinates of the point that will become the origin
    :param config_point: absolute coordinates of the point that makes a right triangle with "point" and "origin" in a desired plane
    :return: coordinates of the point in the new coordinate system
    """
    # config_point is the point that makes a right triangle with "point" and "origin"
    negative_x = True if point[0]-origin[0] < 0 else False
    negative_y = True if point[1]-origin[1] < 0 else False

    x_coordinate = sqrt((point[0]-config_point[0])**2+(point[1]-config_point[1])**2+(point[2]-config_point[2])**2)
    y_coordinate = sqrt((config_point[0]-origin[0])**2+(config_point[1]-origin[1])**2+(config_point[2]-origin[2])**2)

    if negative_x:
        x_coordinate *= -1
    if negative_y:
        y_coordinate *= -1

    return x_coordinate, y_coordinate


def calculate_distance(a, b, focus, angle, length, angle_in_degrees=True):
    """
    Calculates distance from origin to middle of an object of given length rotated by a given angle alpha.
    --> Works only in specific coordinate systems

    :param a: x-coordinate of the first point of the image
    :param b: x-coordinate of the second point of the image
    :param focus: focus length (y-coordinate of the points of the image)
    :param angle: turn angle of the real object
    :param length: length of the initial object
    :return: distance from origin to middle of the object
    """
    if angle_in_degrees:
        angle = radians(angle)

    n = (length * b) / (b - a) * (sin(angle)/focus + cos(angle)/b)
    m = (length*cos(angle) + a * n) / b

    xa, ya = a * n, focus * n
    xb, yb = b * m, focus * m

    xm, ym = 0.5 * (xa + xb), 0.5 * (ya + yb)

    return sqrt(xm**2 + ym**2)


def translate_origin_to_canon_wrongone(coordinates, offset: tuple=(0, 0)):  # FIXME: DOES NOT DO THE TRICK
    """
    Makes the origin coincide with the canon by translating the zero

    For now works only with x- and y-coordinates

    :param coordinates: array of coordinates of points that need to be translated
    :param offset: coordinates of the canon relatively to the camera (IN PIXELS)
    :return: new set of coordinates
    """
    translated = np.empty((4,2), np.int32)  # Creates an empty 2-d array 4-by-2 (4 rows, 2 columns)

    for i, coords in enumerate(coordinates):
        # Fills in 4 rows with translated coordinates
        translated[i] = np.array([[coords[0]-offset[0], coords[1]-offset[1]]])
    
    return translated


#############################
# Translate origin to canon #
#############################

def angle_with_cosine_theorem(a, b, c):
    return acos((a**2+b**2-c**2)/(2*a*b))


def c_with_cosine_theorem(a, b, angle):
    return sqrt(a**2+b**2-2*a*b*cos(angle))

def translate_origin_to_canon(d, M_coords, O_coords, focus_length=(3.04*10**(-3))) -> tuple:
    """
    Gives data needed to aim the canon itself: both parts of the angle and the distance from the canon

    :param d: distance from the camera (IN METERS)
    :param M_coords: coordinates of middle of the image (IN METERS)
    :param O_coords: coordinates of the camera when the canon is the origin (IN METERS)
    :param focus_length: focal length of the camera (IN METERS)
    :return: tuple: (x-angle, y-angle, distance from canon)
    """
    O = O_coords  # Coordinates of the camera

    # Coordinates of the point M' in the coordinate system with canon as origin
    x_M = M_coords[0] + O[0]
    z_M = M_coords[2] + O[2]
    y_M = O_coords[1] - focus_length

    m_accent = (x_M, y_M, z_M)

    # print('***WARNING! DEBUG MODE \\\ COORDINATE TRANSLATION***')
    # m_accent = M_coords  # Debug mode (un)comment if needed

    OM = d
    Q = (0, 0, 0)  # Coordinates of the canon, better leave as (0, 0, 0)

    # Refer to geogebra for clarity
    OM_accent = length_3d(O, m_accent)
    QM_accent = length_3d(Q, m_accent)
    OQ = length_3d(O, Q)

    QOM_accent = degrees(angle_with_cosine_theorem(OQ, OM_accent, QM_accent))

    MOQ = 180 - QOM_accent

    QM = c_with_cosine_theorem(OM, OQ, radians(MOQ))

    OI = OM * cos(asin((m_accent[2]-O[2])/OM_accent))

    MI = sqrt(OM**2-OI**2)

    if m_accent[2] - O[2] > 0:  # Point M lies under the point I
        MK = O[2] - MI  # MK is essentially the z-coordinate of point M
    else:
        MK = O[2] + MI

    KQM = degrees(asin(MK/QM))  # vertical angle

    QK = sqrt(QM**2-MK**2)

    J = (O[0], O[1], Q[2])

    QJ = length_3d(Q, J)

    JK = OI

    xQJ = degrees(atan(abs(J[1]/J[0])))

    try:
        JQK = degrees(angle_with_cosine_theorem(QJ, QK, JK))
    except ValueError:
        JQK = 180
    finally:
        if degrees(atan((m_accent[0]-O[0])/focus_length)) > (90-xQJ):
            yQK = JQK + xQJ - 270  # horizontal angle
        else:
            yQK = 90 - (JQK - xQJ)
    
    return (yQK, KQM, QM)


# Translate coordinates of the image
def translate_image_point(point_image, current_resolution=(640, 480), sensor_size=(3.68*10**(-3), 2.76*10**(-3))):
    """
    Translates the coordinates of a point in image to "real" coordinates

    :param point_image: two coordinates of a point
    :param current_resolution: resolution in which image has been taken
    :param sensor_size: length and width of the sensor of the camera
    """

    # Coordinates of the point (here point P) if the middel of the image were the origin \ IN PIXELS
    x_P_px = current_resolution[0]//2 - point_image[0]
    y_P_px = -point_image[1] + current_resolution[1]//2

    # What part of the sensor does it "cover"
    x_percentage = x_P_px / current_resolution[0]
    y_percentage = y_P_px / current_resolution[1]

    # "Real" coordinates of the point P \ IN METERS
    x_P_m = x_percentage * sensor_size[0]
    y_P_m = y_percentage * sensor_size[1]

    # Return the coordinates
    return (x_P_m, y_P_m)


# Angle with drag
def find_angle_with_drag(x: float, y: float, m: float, v0: float,
                            k: float, g: float = 9.81, give_degrees: bool = True) -> float:
    """
    Calculates the angle needed to hit a point with given x- and y-coordinates.
    The calculations take drag (air resistance) into account.
    The functions are however far to complex for an exact solution, which is why it is calculated numerically.
    
    :param x: x-coordinate of the point
    :param y: y-coordinate of the point
    :param m: mass of the projectile
    :param v0: initial velocity of the projectile
    :param k: complete drag coefficient, i.e. k = 1/2*air_density*drag_coefficient*area
    :param g: gravitational acceleration
    :param give_degrees: whether result should be in degrees, if False then radians are given
    :return: angle
    """
    alpha = s.Symbol('alpha')

    # Function of time of x: time that corresponds with the given x-coordinate
    t = (m/k) * (s.exp(x*k/m)-1)/(v0*s.cos(alpha))

    # Time and height of the highest point
    t0 = s.sqrt(m/(g*k))*s.atan(v0*s.sin(alpha)*s.sqrt(k/(g*m)))
    h0 = 1/2*m/k*s.ln((v0*s.sin(alpha))**2*k/(g*m)+1)

    # Two equations for two parts of the y-coordinate movement
    y_up = -y + m/k*s.ln(s.cos(t*s.sqrt(g*k/m)-s.atan(v0*s.sin(alpha)*s.sqrt(k/(g*m))))) + h0
    y_down = -y + h0 - m/k*s.ln(s.cosh((t-t0)*s.sqrt(g*k/m)))
    
    # Lambdify in order to be able to use SciPy
    y_up0 = s.lambdify(alpha, y_up)
    y_down0 = s.lambdify(alpha, y_down)

    # Find the roots numerically; works only with f(x)=0, hence -y in equations is needed
    alpha1 = s.nsolve(y_up, (0, pi/4), solver='bisect')
    alpha2 = s.nsolve(y_down, (0, pi/4), solver='bisect')

    # Introduce the equation for x coordinate
    t_ = s.Symbol('t_')
    x_ = m/k*s.ln(v0*s.cos(alpha)*k/m*t_ + 1)

    # Find the time which corresponds with the highest point for both angles
    t01 = t0.evalf(subs={alpha: alpha1})
    t02 = t0.evalf(subs={alpha: alpha2})

    # Find the x-coordinate of the highest point for both angles
    x01 = x_.evalf(subs={t_: t01, alpha: alpha1})
    x02 = x_.evalf(subs={t_: t02, alpha: alpha2})

    # Translate to degrees if needed
    if give_degrees:
        alpha1 = round(degrees(alpha1), 4)
        alpha2 = round(degrees(alpha2), 4)

    # Check to which part the given x belongs and hence which angle should be used
    if x <= x01:
        print(' --> We use the first half')
        return alpha1
    if x >= x02:
        print(' --> We use the second half')
        return alpha2




if __name__ == '__main__':
    config = ConfigParser()
    config_file = 'cfg.ini'
    config.read(config_file)

    d = 0.6
    O_coords = (0.03, -0.11, 0.04)
    M_coords = (
        0.0002825,
        -0.00304,
        -0.000563
    )


    print(translate_origin_to_canon(d, M_coords, O_coords, focus_length=3.04*10**(-3)))