#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Bool

# [navigation_control] uses (course_correct) <course>, 
# (collision_state) <collision>, (end_of_field_state) <end>, 
# (turning_correct) <turning> and publishes all four wheel speeds 
# to (wheel_cmd_speed) <wheel>

# This node is a TEMPLATE

def course_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def collision_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def end_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def turning_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def navigation_control():
    # Publisher to topic crop_lcation
    pub_wheel = rospy.Publisher('wheel_cmd_speed', String, queue_size=10)

    # Subscribes to topic 
    sub_course = rospy.Subscriber('course_correct', String, course_callback)

    # Subscribes to topic 
    sub_collision = rospy.Subscriber('collision_state', Bool, collision_callback)

    # Subscribes to topic 
    sub_end = rospy.Subscriber('end_of_field_state', String, end_callback)
    
    # Subscribes to topic 
    sub_turning = rospy.Subscriber('turning_correct', String, turning_callback)

    rospy.init_node('navigation_control')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_wheel.publish(message)
        #rospy.spin_once() (get_spline) <spline>
        rate.sleep()
        

if __name__ == '__main__':
    try:
        navigation_control()
    except rospy.ROSInterruptException:
        pass
