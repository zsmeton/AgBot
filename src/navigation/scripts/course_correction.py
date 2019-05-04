#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Float32
from controller_pid import PID
import time

# [course_correction] adjusts heading based on (crop_location) 
# and gives an adjustment to the wheel speed (course_correct)
#
# Input [left_crop, middle_crop, right_crop]
# e(t) = 0 - middle_crop
# left_wheel_pwd/speed = left_wheel_pwd/speed + u(t)
# right_wheel_pwd/speed = right_wheel_pwd/speed - u(t)

# variables
u_t = 0
pid_last_runtime = time.time()
# paramters
k_p = 5
k_i = 3
k_d = 3


def get_parameters():
    """ Sets the ros parameters if they exist"""
    if rospy.has_param('~k_p'):
        global k_p
        k_p = rospy.get_param('~k_p')
    if rospy.has_param('~k_i'):
        global k_i
        k_i= rospy.get_param('~k_i')
    if rospy.has_param('~k_d'):
        global k_d
        k_d = rospy.get_param('~k_d')


# TODO: Look into adding a reset sytem to reset the PID so the robot turning doesn't fuck the system
def crop_callback(data):
    # Check for correct input dimensions
    assert(len(data.data) == 3), "Course Correction was designed for three crop locations"
    # Run PID on input
    global u_t, pid_last_runtime
    e_t = 0-data.data[1]  # error is based on center value
    dt = time.time() - pid_last_runtime
    u_t = PID(error=e_t, dt=dt, k_p=k_p, k_i=k_i, k_d=k_d)
    pid_last_runtime = time.time()


def course_correction():
    # Publisher to topic crop_lcation
    pub_course = rospy.Publisher('course_correct', Float32, queue_size=10)

    # Subscribes to topic lidar
    sub_crop = rospy.Subscriber('crop_location', Float32MultiArray, crop_callback)

    rospy.init_node('course_correction')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        correction = Float32()
        correction.data = u_t
        pub_course.publish(correction)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        course_correction()
    except rospy.ROSInterruptException:
        pass