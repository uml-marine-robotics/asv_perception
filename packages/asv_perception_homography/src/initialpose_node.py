#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
#from tf.transformations import quaternion_from_euler

import asv_perception_utils as utils

class initialpose_node(object):

    def __init__(self):
        
        self.node_name = rospy.get_name()

        # test for navsat_transform_node
        self.pub_navsat = rospy.Publisher( "/initialpose", Odometry, latch=True, queue_size=1 )

        # https://answers.ros.org/question/295803/unable-to-use-navsat_transform_node/
        msg = Odometry()
        msg.header.stamp=rospy.Time.now()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.orientation.x=msg.pose.pose.orientation.y=msg.pose.pose.orientation.z=0
        msg.pose.pose.orientation.w=1
        self.pub_navsat.publish(msg)
    
    
if __name__ == "__main__":

    try:
        rospy.init_node(initialpose_node.__name__)
        n = initialpose_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
