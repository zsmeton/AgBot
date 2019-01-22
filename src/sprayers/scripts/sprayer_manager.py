#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [sprayer_manager] uses (plant_stream). For weeds it then publishes 
# desired weed sprayer orientations, for closest weed to that sprayer, 
# to (weed_sprayer_teleop) <weed_teleop>. It continues to track the location of 
# the weed using (plant_stream), when the weed is in the correct 
# position to be sprayed it triggers the weed sprayer for 
# PARAM:spray_time using (weed_sprayer_trigger) <weed_trigger>. For corn it just 
# sprays when the plant is in the correct position for PARAM:spray_time 
# using (crop_sprayer_trigger) <crop_trigger>. 

# This node is a TEMPLATE

def plant_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def sprayer_manager():
    # Publisher to topic
    pub_weed_teleop = rospy.Publisher('weed_sprayer_teleop', String, queue_size=10)
    # Publisher to topic
    pub_weed_trigger = rospy.Publisher('weed_sprayer_trigger', String, queue_size=10)
    # Publisher to topic
    pub_crop_trigger = rospy.Publisher('crop_sprayer_trigger', String, queue_size=10)

    # Subscribes to topic
    sub_plant = rospy.Subscriber('plant_stream', String, plant_callback)

    rospy.init_node('sprayer_manager')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_weed_teleop.publish(message)
        pub_weed_trigger.publish(message)
        pub_crop_trigger.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        sprayer_manager()
    except rospy.ROSInterruptException:
        pass