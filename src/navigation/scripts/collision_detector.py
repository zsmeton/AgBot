#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool

# [collision_detector] uses (ultrasonic) to detect when 
# ultrasonic is activated. If bumper is activated robot stops, turn 
# on safety lights, and waits until (safety_button) publishes a 
# True. Others can find what the robot state is through 
# (collision_state)

collision_state = False

def ultrasonic_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " Received ultrasonic state: %s", data.data)
    if (data.data):
        global collision_state
        collision_state = True  # Activate collision state 

def safety_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " Received safety button press.")
    if (data.data):
        global collision_state
        collision_state = False  # Deactivate collision state

def collision_detector():
    # Publisher to topic collision_state
    pub_collision = rospy.Publisher('collision_state', Bool, queue_size=10)

    # Subscribes to topic bumper_button
    sub_ultrasonic = rospy.Subscriber('ultrasonic', Bool, ultrasonic_callback)

    # Subscibes to topic safety_button
    sub_safety = rospy.Subscriber('safety_button', Bool, safety_callback)

    rospy.init_node('collision_detector')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes collision state var to collision_state
        pub_collision.publish(collision_state)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        collision_detector()
    except rospy.ROSInterruptException:
        pass
