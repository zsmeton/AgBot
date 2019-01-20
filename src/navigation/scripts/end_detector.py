#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [end_detector] detects when at end of field using (lidar) 
# and published boolean to (end_of_field_state)

# This node is a TEMPLATE

def lidar_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def end_detector():
    # Publisher to topic crop_lcation
    pub_end = rospy.Publisher('end_of_field_state', String, queue_size=10)

    # Subscribes to topic lidar
    sub_lidar = rospy.Subscriber('lidar', String, lidar_callback)

    rospy.init_node('end_detector')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_end.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        end_detector()
    except rospy.ROSInterruptException:
        pass