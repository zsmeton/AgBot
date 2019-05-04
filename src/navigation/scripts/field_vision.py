#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
import numpy as np
from numpy.random import randn
import scipy as sc
import scipy.optimize
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from math_extension import variable_mapping
from field_vision_functions import *
import os

os.path.abspath(".")


def isnan(x): return type(x) is float and x != x


def isinf(x): inf = 1e5000; return x == inf or x == -inf


# [field_vision] identifies location of crops vs path using
# (lidar) and publishes location relative to the center of the
# robot for all three crops to (crop_location)

# Parameters and constants
DEBUG = False
MODALS = 3
call_back = False
desired_angle_min = -0.6
desired_angle_max = 0.6
x = np.linspace(-10, 10)
y = three_peaks(x, -5 + randn(), randn(), 5 + randn())
crop_location_guess = [-5, 0, 5]  # we guess that our rows are at -5 and 5 relative to the heading based on prior info
vvariable_mapping = np.vectorize(variable_mapping)

# TODO: Add ros_params to set desired_angle_min and max
def get_parameters():
    """ Sets the ros parameters if they exist"""
    if rospy.has_param('~row_guess'):
        global crop_location_guess
        crop_location_guess = rospy.get_param('~row_guess')


def lidar_callback(msg):
    global call_back
    call_back = True
    # Create a vector of angles from the minimum angle to the maximum angle of the length of the message data
    angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
    # Set radius to the data from the message
    radius = []
    counter = 0
    for range in msg.ranges:
        if angles[counter] > desired_angle_max or angles[counter] < desired_angle_min:
            angles = np.delete(angles, counter)
            counter -= 1
        elif isinf(range) or isnan(range):
            angles = np.delete(angles, counter)
            counter -= 1
        else:
            radius.append(range)
        counter += 1
    radius = np.array(radius)
    # Sets x and y to the coordinates of the scanner points in meters
    global x
    x = np.sin(angles) * radius
    y = np.cos(angles) * radius

    # Map the values of y from 0 to 1 for better multimodal analysis
    try:
        global y
        y = vvariable_mapping(y, y.min(), y.max(), 0, 1)
    except ZeroDivisionError:
        print("Could not properly map lidar data, using old lidar data for detection")



def crop_location_to_ros_msg(crop_location):
    row_multi_array = Float32MultiArray()
    row_multi_array.layout.dim.append(MultiArrayDimension())
    row_multi_array.layout.dim[0].label = "row location"
    row_multi_array.layout.dim[0].size = np.array(crop_location).size
    row_multi_array.layout.dim[0].stride = np.array(crop_location).size
    row_multi_array.layout.data_offset = 0
    row_multi_array.data = np.array(crop_location).tolist()
    return row_multi_array


def field_vision():
    # Publisher to topic crop_lcation
    pub_crop = rospy.Publisher('crop_location', Float32MultiArray, queue_size=10)

    # Subscribes to topic lidar
    sub_lidar = rospy.Subscriber('scan', LaserScan, lidar_callback)

    rospy.init_node('field_vision')
    # TODO: Try faster rates
    rate = rospy.Rate(1)  # 1hz

    # set up plotting for visualization
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    lidar_line, = ax.plot(x, y)
    crop_location_line = ax.scatter(crop_location_guess, np.zeros(len(crop_location_guess)), c='r', marker='+')
    #model_line, = ax.plot(x, y)

    while not rospy.is_shutdown():
        # Align the data for modeling
        xa, ya, sclr = align_data(tuple(x), tuple(y))
        # Model the data
        try:
            crop_guess_temp = find_rows(xa, ya, crop_location_guess)
        except RuntimeError:
            print("Could not find optimal parameters")
            continue

        a, b, c = crop_guess_temp # get unscaled corp locations for data visualization
        if DEBUG:
            # Plot aligned data and model
            plt.scatter(xa, ya)
            plt.plot(x, three_peaks(x, a, b, c), '--r')
            plt.scatter(crop_guess_temp, np.zeros(len(crop_guess_temp)), c='r', marker='+')
            plt.title("Alinged data")
            plt.show()


        # Convert data back
        crop_guess_temp = convert_back(crop_guess_temp, sclr)
        # Only publish once lidar data comes in
        if call_back:
            pub_crop.publish(crop_location_to_ros_msg(crop_guess_temp))

        # Plot the data for visualization
        lidar_line.set_xdata(x)
        lidar_line.set_ydata(y)
        lidar_line.set_xdata(x)
        #lidar_line.set_ydata(convert_back(three_peaks(xa,a,b,c),sclr))
        crop_location_line.set_offsets(np.c_[crop_guess_temp, np.zeros(len(crop_guess_temp))])
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()

        rate.sleep()


if __name__ == '__main__':
    try:
        field_vision()
    except rospy.ROSInterruptException:
        pass
