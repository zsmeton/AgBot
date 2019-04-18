#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32MultiArray,MultiArrayDimension, String
import numpy as np
from numpy.random import randn
import scipy as sc
import scipy.optimize
from sensor_msgs.msg import LaserScan

# [field_vision] identifies location of crops vs path using 
# (lidar) and publishes location relative to the center of the
# robot for all three crops to (crop_location)

def place_peaks(x, a, b, r=1):
    """
    place_peaks(x,a,b,r=1)
        x : array like
        a : float
        b : float
        r : float
    constructs a bi-modal negative absolute value at locations
    x = a, and x = b, with radius r: ____/\\_______/\\__ sort of shape
                                          a        b
    """
    n = len(x)
    y = np.zeros(n)
    for i in range(n):
        if (a - r <= x[i] and x[i] <= a + r):
            y[i] = r - abs(x[i] - a)
        elif (b - r <= x[i] and x[i] <= b + r):
            y[i] = r - abs(x[i] - b)
    return -y + 0.01 * randn(n)


# Parameters and constants
PARAM_DEBUG = False
x = np.linspace(-10, 10)
y = place_peaks(x, -5 + randn(), 5 + randn(), 1)
crop_location_guess = [-5, 5]  # we guess that our rows are at -5 and 5 relative to the heading based on prior info


def bi_modal_gaussian(crop_location_guess):
    '''
    g(crop_location_guess)
        crop_location_guess : length 2 array like
    evaluates a bi-modal gaussian with respect to 2 parameters,
    each specifying the center of one of the modes.
    '''
    return -np.exp(-(x - crop_location_guess[0]) ** 2) - np.exp(-(x - crop_location_guess[1]) ** 2)


def residual_vector(crop_location_guess):
    """
    f(crop_location_guess)
        crop_location_guess : length 2 array like
    returns the residual vector g(crop_location_guess) - y.
    Is the objective function for minimization.
    """
    return bi_modal_gaussian(crop_location_guess) - y


def jacobian(crop_location_guess):
    """
    J(crop_location_guess)
        crop_location_guess : length 2 array like
    returns the jacobian of the residual vector w/ respect to crop_location_guess.
    Useful for efficient optimization.
    """
    A = np.zeros((len(x), 2))
    A[:, 0] = -2 * (x - crop_location_guess[0]) * np.exp(-(x - crop_location_guess[0]) ** 2)
    A[:, 1] = -2 * (x - crop_location_guess[1]) * np.exp(-(x - crop_location_guess[1]) ** 2)
    return A


def find_rows(crop_location_guess):
    '''
    find_rows(crop_location_guess)
        crop_location_guess : length 2 array like, initial guess for row locations
    returns precise estimate of row location.
    '''
    return sc.optimize.least_squares(residual_vector, crop_location_guess, jac=jacobian, method='lm').x  # lm is used for efficiency!



def get_parameters():
    """ Sets the ros parameters if they exist"""
    params_exits = False
    if rospy.has_param('~row_guess'):
        global crop_location_guess
        crop_location_guess = rospy.get_param('~row_guess')


def lidar_callback(msg):
    global y
    global x
    # Create a vector of angles from the minimum angle to the maximum angle of the length of the message data
    angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
    # Set radius to the data from the message
    radius = np.array(msg.ranges)
    # Sets x and y to the coordinates of the scanner points in meters
    x = np.cos(angles) * radius
    y = np.sin(angles) * radius


def field_vision():
    # Publisher to topic crop_lcation
    pub_crop = rospy.Publisher('crop_location', Float32MultiArray, queue_size=10)

    # Subscribes to topic lidar
    sub_lidar = rospy.Subscriber('lidar', LaserScan, lidar_callback)
    
    rospy.init_node('field_vision')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Find the rows
        global crop_location_guess
        crop_location_guess = find_rows(crop_location_guess)  # we find the actual position
        # Publishes crop_location_guess to crop_location
        row_multi_array = Float32MultiArray()
        row_multi_array.layout.dim.append(MultiArrayDimension())
        row_multi_array.layout.dim[0].label = "row location"
        row_multi_array.layout.dim[0].size = np.array(crop_location_guess).size
        row_multi_array.layout.dim[0].stride = np.array(crop_location_guess).size
        row_multi_array.layout.data_offset = 0
        row_multi_array.data = np.array(crop_location_guess).tolist()
        pub_crop.publish(row_multi_array)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        field_vision()
    except rospy.ROSInterruptException:
        pass