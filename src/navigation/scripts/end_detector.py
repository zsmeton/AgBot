#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import numpy as np

# [end_detector] detects when at end of field using (lidar) 
# and published boolean to (end_of_field_state)

# This node is a TEMPLATE
message = True
previous = []
counter = 0;
def lidar_callback(msg):
    global message
    global previous
    global counter
    print(msg.ranges[180*4])
    if(counter == 0):
        for p in range(721):
            previous.append(msg.ranges[p])
            counter = 1
        print(previous[180*4])
    elif(counter != 0):
        check_two = 0
        for i in range(721):
            check_two = abs(msg.ranges[i] - previous[i])
            #Crops are around 6 - 12 inches tall
            #1 inch = 0.0254 m   |   0.3048 m = 12 inches

            #I am following the measurements said by AgBot,
            #We might need to change this in the future.cle

            if(check_two > 0.3048):
                message = False
                break
            elif(check_two < 0.3048):
                message = True
            elif(i == 180*4):
                print(check_two)
            counter = 0
    print(message)
    print("  ")

def end_detector():
    # Publisher to topic crop_lcation
    pub_end = rospy.Publisher('end_of_field_state', Bool, queue_size=10)
    # Subscribes to topic /scan
    sub_lidar = rospy.Subscriber('/scan', LaserScan, lidar_callback)

    rospy.init_node('end_detector')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        global message
        pub_end.publish(message)
        #rospy.loginfo(message)
        #rospy.spin_once()
        rate.sleep()
        
if __name__ == '__main__':
    try:
        end_detector()
    except rospy.ROSInterruptException:
        pass