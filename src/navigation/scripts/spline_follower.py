#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [spline_follower] records robots movements using (compass) <compass>, 
# (robot_pos_delta) <position>, (gps) <gps>, and (get_spline) <spline> then 
# publishes heading and speed to (turning_correct) <turning>

# This node is a TEMPLATE

def compass_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def position_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def gps_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def spline_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def spline_follower():
    # Publisher to topic 
    pub_turning = rospy.Publisher('turning_correct', String, queue_size=10)

    # Subscribes to topic 
    sub_compass = rospy.Subscriber('compass', String, compass_callback)

    # Subscribes to topic 
    sub_position = rospy.Subscriber('robot_pos_delta', String, position_callback)

    # Subscribes to topic 
    sub_gps = rospy.Subscriber('gps', String, gps_callback)
    
    # Subscribes to topic 
    sub_spline = rospy.Subscriber('get_spline', String, spline_callback)
    
    rospy.init_node('spline_follower')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_turning.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        spline_follower()
    except rospy.ROSInterruptException:
        pass