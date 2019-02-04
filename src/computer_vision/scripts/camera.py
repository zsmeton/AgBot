#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [camera] takes picture and publishes to (images) <images>

# This node is a TEMPLATE


def camera():
    # Publisher to topic
    pub_images = rospy.Publisher('images', String, queue_size=10)

    rospy.init_node('camera')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_images.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass
