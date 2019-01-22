#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [plant_vision] takes the pictures from (images) runs detection 
# on the image then publishes weed classification certainty; 
# location, relative to the camera;  and camera number to 
# (raw_weeds) <raw>

# This node is a TEMPLATE

def images_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def plant_vision():
    # Publisher to topic
    pub_raw = rospy.Publisher('raw_weeds', String, queue_size=10)

    # Subscribes to topic
    sub_images = rospy.Subscriber('images', String, images_callback)

    rospy.init_node('plant_vision')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_raw.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        plant_vision()
    except rospy.ROSInterruptException:
        pass