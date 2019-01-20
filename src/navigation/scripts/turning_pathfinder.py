#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [turning_pathfinder] takes robot parameters and (field_information) <field> 
# to create a spline which the robot will follow to turn. Spline 
# is accessed by (get_spline) <spline>

# This node is a TEMPLATE

def field_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def turning_pathfinder():
    # Publisher to topic crop_lcation
    pub_spline = rospy.Publisher('get_spline', String, queue_size=10)

    # Subscribes to topic lidar
    sub_field = rospy.Subscriber('field_information', String, field_callback)

    rospy.init_node('turning_pathfinder')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_spline.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        turning_pathfinder()
    except rospy.ROSInterruptException:
        pass