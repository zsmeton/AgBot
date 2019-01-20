#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [collision_detector] uses (bumper_button) to detect when 
# button is activated. If bumper is activated robot stops, turn 
# on safety lights, and waits until (safety_button) publishes a 
# true. Others can find what the robot state is through 
# (collision_state)

# This node is a TEMPLATE

def bump_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def safety_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def collision_detector():
    # Publisher to topic collision_state
    pub_collision = rospy.Publisher('collision_state', String, queue_size=10)

    # Subscribes to topic bumper_button
    sub_bump = rospy.Subscriber('bumper_button', String, bump_callback)

    # Subscibes to topic safety_button
    sub_safety = rospy.Subscriber('safety_button', String, safety_callback)

    rospy.init_node('collision_detector')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_collision.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        collision_detector()
    except rospy.ROSInterruptException:
        pass