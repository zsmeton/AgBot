#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [course_correction] adjusts heading based on (crop_location) 
# and gives the heading and speed under (course_correct)

# This node is a TEMPLATE

def crop_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def course_correction():
    # Publisher to topic crop_lcation
    pub_course = rospy.Publisher('course_correct', String, queue_size=10)

    # Subscribes to topic lidar
    sub_crop = rospy.Subscriber('crop_location', String, crop_callback)

    rospy.init_node('course_correction')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_course.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        course_correction()
    except rospy.ROSInterruptException:
        pass