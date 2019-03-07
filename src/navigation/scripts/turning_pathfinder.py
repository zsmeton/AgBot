#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32, Float32MultiArray, String

#   [turning_pathfinder] maps and contains information on the row locations 
#   and distances from the center of the bot using data from (crop_location) <crop> 
#   to provide a set of wheel speeds for the outer and inner wheel accessed 
#   through (turning_speed) <turning> 


x_start = 0
x_len = 2
turn_radius = 0
row_seperation_avg = None
row_skip = 1
outer_speed_max = 2
pub_params = None

PARAM_DEBUG = False

def field_callback(data):
    """ Compute turn_radius and update global node var. """
    rospy.loginfo(rospy.get_caller_id() + " Received field info: %s", data.data)
    xs = data.data[x_start:x_start+x_len]  # Select x segment
    c = [(xs[i] + xs[i+1])/2. for i in range(0, x_len, 2)]  # Compute center points
    dists = [r-l for r, l in zip(c[1:], c[:-1])]  # Compute distances between centers
    avg = sum(dists)/len(dists)
    global turn_radius
    turn_radius = 1.5*avg

def get_parameters():
    """ Sets the ros parameters if they exist"""
    params_exits = False
    if rospy.has_param('~row_skip'):
        global row_skip
        row_skip = rospy.get_param('~row_skip')
        params_exits = True
    if rospy.has_param('~outer_speed_max'):
        global outer_speed_max
        outer_speed_max = rospy.get_param('~outer_speed_max')
        params_exits = True
    if PARAM_DEBUG:
        global pub_params
        pub_params.publish(rospy.get_name()+ "  " +str(params_exits))


def turning_pathfinder():
    
    # Publisher to topic turn_radius
    pub_turn = rospy.Publisher('turn_radius', Float32, queue_size=10)

    if PARAM_DEBUG:
        global pub_params
        pub_params = rospy.Publisher('turning_params', String, queue_size=10)

    # Subscribes to topic lidar
    sub_field = rospy.Subscriber('crop_location', Float32MultiArray, field_callback)

    rospy.init_node('turning_pathfinder')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        global turn_radius
        pub_turn.publish(turn_radius)
        get_parameters()

        rate.sleep()
        

if __name__ == '__main__':
    try:
        turning_pathfinder()
    except rospy.ROSInterruptException:
        pass
