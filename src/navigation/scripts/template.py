#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# {Node Description}

# This node is a TEMPLATE

def {topic short hand}_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def {node name}():
    # Publisher to topic crop_lcation
    pub_{topic short hand} = rospy.Publisher('{topic}', String, queue_size=10)

    # Subscribes to topic lidar
    sub_{topic short hand} = rospy.Subscriber('{topic}', String, {topic short hand}_callback)

    rospy.init_node('{node name')
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # Publishes "TEST" to course_correct
        message = "TEST"
        pub_{topic short hand}.publish(message)
        #rospy.spin_once()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        {node name}()
    except rospy.ROSInterruptException:
        pass