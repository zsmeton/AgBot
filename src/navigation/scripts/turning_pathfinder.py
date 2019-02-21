#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32, Float32MultiArray

# [turning_pathfinder] takes robot parameters and (field_information) <field> 
# to provide a radius (in meters) at which the robot will turn. Radius is 
# accessed by (turn_radius) <turn>

x_start = 0
x_len = 6
turn_radius = 0

def field_callback(data):
    """ Compute turn_radius and update global node var. """
    rospy.loginfo(rospy.get_caller_id() + " Received field info: %s", data.data)
    xs = data.data[x_start:x_start+x_len]  # Select x segment
    c = [(xs[i] + xs[i+1])/2. for i in range(0, x_len, 2)]  # Compute center points
    dists = [r-l for r, l in zip(c[1:], c[:-1])]  # Compute distances between centers
    avg = sum(dists)/len(dists)
    global turn_radius
    turn_radius = 1.5*avg


def turning_pathfinder():
    # Publisher to topic turn_radius
    pub_turn = rospy.Publisher('turn_radius', Float32, queue_size=10)

    # Subscribes to topic lidar
    sub_field = rospy.Subscriber('field_information', Float32MultiArray, field_callback)

    rospy.init_node('turning_pathfinder')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        pub_turn.publish(turn_radius)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        turning_pathfinder()
    except rospy.ROSInterruptException:
        pass
