#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

# [end_detector] detects when at end of field using (lidar) 
# and published boolean to (end_of_field_state)

# This node is a TEMPLATE
i = 0
avg_diff = []
distance = [0]
message = False
def lidar_callback(msg):
    #rospy.loginfo(data.data)
    global i
    global avg_diff
    global distance
    print 'Value at 0*: '
    print msg.ranges[0]
    print 'Value at 90*: '
    print msg.ranges[360]
    print 'Value at 180*: '
    print msg.ranges[719]
    print ' '

def end_detector():
    # Publisher to topic crop_lcation
    pub_end = rospy.Publisher('end_of_field_state', Bool, queue_size=10)
    # Subscribes to topic /scan
    sub_lidar = rospy.Subscriber('/scan', LaserScan, lidar_callback)

    rospy.init_node('end_detector')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        #pub_end.publish(message)
        #rospy.loginfo(sub_lidar)
        #rospy.spin_once()
        rate.sleep()
        
if __name__ == '__main__':
    try:
        end_detector()
    except rospy.ROSInterruptException:
        pass