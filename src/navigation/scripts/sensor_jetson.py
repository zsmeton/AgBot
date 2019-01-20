#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [sensor_jetson] pulls lidar information and publishes to 
# (lidar) <lidar>


# This node is a TEMPLATE

def sensor_jetson():
    # Publisher to topic crop_lcation
    pub_lidar = rospy.Publisher('lidar', String, queue_size=10)

    rospy.init_node('sensor_jetson')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_lidar.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        sensor_jetson()
    except rospy.ROSInterruptException:
        pass