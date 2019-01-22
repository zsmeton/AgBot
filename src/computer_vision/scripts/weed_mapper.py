#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# [weed_mapper] takes in (raw_weeds) <raw>, translates the location to a global position, 
# plots the weed in a 2d map. If weeds overlap, combine classification certainty, 
# use unique ids to identify image groupings. Continues to track location of weeds 
# based on (robot_pos_delta) <pos> and (gps) <gps> . Publishes weed information to 
# (plant_stream) <plant> 
# with the: id; greatest classification, in this case simplified to weed/corn; and 
# location, relative to the closest sprayer. Removes weeds from map and stream after 
# the robot has passed over it.  [weed_mapper] also deducts classifications by 
# PARAM:classification_deduction to eliminate flukes, and if the classification is 
# below PARAM:classification_min the plant is deleted from map and stream

# This node is a TEMPLATE

def raw_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def gps_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def pos_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def weed_mapper():
    # Publisher to topic
    pub_plant = rospy.Publisher('plant_stream', String, queue_size=10)

    # Subscribes to topic
    sub_raw = rospy.Subscriber('raw_weeds', String, raw_callback)
    sub_gps = rospy.Subscriber('gps', String, gps_callback)
    sub_pos = rospy.Subscriber('robot_pos_delta', String, pos_callback)

    rospy.init_node('weed_mapper')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_plant.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        weed_mapper()
    except rospy.ROSInterruptException:
        pass