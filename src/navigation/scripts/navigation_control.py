#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool, Float32, Float32MultiArray, MultiArrayDimension

# [navigation_control] uses (course_correct) <course>, 
# (collision_state) <collision>, (end_of_field_state) <end>, 
# (turn_radius) <turning> and publishes all four wheel speeds
# to (wheel_cmd_speed) <wheel>

course_val = 0.
collision_state = False
end_state = False
turn_val = 0.

DEFAULT_SPEED = 0.5
WHEEL_DIST = 0.25


def course_callback(data):
    """ Get course correction value """
    global course_val
    course_val = data.data


def collision_callback(data):
    """ Update collision state """
    global collision_state
    collision_state = data.data


def end_callback(data):
    """ Update end state """
    global end_state
    end_state = data.data


def turning_callback(data):
    """ Get turning value """
    global turn_val
    turn_val = data.data


def generate_message():
    """ Generates multiarray message """
    msg = Float32MultiArray()
    lay = msg.layout
    lay.dim.append(MultiArrayDimension())
    lay.dim[0].label = 'wheel speeds'
    lay.dim[0].size = 2
    lay.dim[0].stride = 2
    lay.data_offset = 0
    msg.data = get_speeds()
    return msg


def get_speeds():
    """ Generates speeds for tank drive train, normalized to [-1, 1] """
    speeds = [DEFAULT_SPEED, DEFAULT_SPEED]  # Default speeds

    if collision_state:  # Collision, set speeds to 0
        speeds = [0., 0.]

    elif end_state:  # End state, set turn
        # Compute turning speeds
        speeds[0] = DEFAULT_SPEED * (1 + WHEEL_DIST / turn_val)
        speeds[1] = DEFAULT_SPEED * (1 - WHEEL_DIST / turn_val)

    else:  # Normal drive, course correction
        # TODO Correct proper side
        if course_val > 0:  # TODO Deadzone
            speeds[0] += course_val
        else:
            speeds[1] -= course_val

    # Filter out-of-bound speeds to [-1, 1]
    speeds = [min(max(speed, -1.), 1.) for speed in speeds]

    return speeds


def navigation_control():
    # Publisher to topic crop_lcation
    pub_wheel = rospy.Publisher('wheel_cmd_speed', Float32MultiArray)

    # Subscribers
    sub_course = rospy.Subscriber('course_correct', Float32, course_callback)
    sub_collision = rospy.Subscriber('collision_state', Bool, collision_callback)
    sub_end = rospy.Subscriber('end_of_field_state', Bool, end_callback)
    sub_turning = rospy.Subscriber('turn_radius', Float32, turning_callback)

    rospy.init_node('navigation_control')

    # TODO Modify rate
    rate = rospy.Rate(5)  # 5 Hz
    
    while not rospy.is_shutdown():
        pub_wheel.publish(generate_message())
        rate.sleep()
        

if __name__ == '__main__':
    try:
        navigation_control()
    except rospy.ROSInterruptException:
        pass
