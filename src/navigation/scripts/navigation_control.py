#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [navigation_control] uses (course_correct) <course>, 
# (collision_state) <collision>, (end_of_field_state) <end>, 
# (get_spline) <spline> and publishes all four wheel speeds 
# to (wheel_cmd_speed) <wheel>

# This node is a TEMPLATE

def course_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def collision_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def end_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def spline_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def navigation_control():
    # Publisher to topic crop_lcation
    pub_wheel = rospy.Publisher('wheel_cmd_speed', String, queue_size=10)

    # Subscribes to topic 
    sub_course = rospy.Subscriber('course_correct', String, course_callback)

    # Subscribes to topic 
    sub_collision = rospy.Subscriber('collision_state', String, collision_callback)

    # Subscribes to topic 
    sub_end = rospy.Subscriber('end_of_field_state', String, end_callback)
    
    # Subscribes to topic 
    sub_spline = rospy.Subscriber('get_spline', String, spline_callback)

    rospy.init_node('navigation_control')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_wheel.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        navigation_control()
    except rospy.ROSInterruptException:
        pass