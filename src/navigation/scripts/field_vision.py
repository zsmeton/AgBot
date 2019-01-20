#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [field_vision] identifies location of crops vs path using 
# (lidar) and publishes location relative to the center of the
# robot in message format ‘[crop left coordinate, crops right coordinate]’
# for all three crops to (crop_location)

# This node is a TEMPLATE

def lidar_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def field_vision():
    # Publisher to topic crop_lcation
    pub_crop = rospy.Publisher('crop_location', String, queue_size=10)

    # Subscribes to topic lidar
    sub_lidar = rospy.Subscriber('lidar', String, lidar_callback)
    
    rospy.init_node('field_vision')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to crop_location
        message = "TEST"
        pub_crop.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        field_vision()
    except rospy.ROSInterruptException:
        pass