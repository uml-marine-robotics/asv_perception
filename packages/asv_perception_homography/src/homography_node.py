#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Header
from asv_perception_common.msg import Homography
import asv_perception_utils as utils

from calibrate import warpMatrix

class homography_node(object):

    def __init__(self):
        
        self.node_name = rospy.get_name()
        
        # YPR in degrees
        self.imu_yaw = 0.   
        self.imu_pitch = 0.
        self.imu_roll = 0.

        self.has_published = False
        
        # publish with latch in case of no IMU/testing/etc
        self.pub = rospy.Publisher( "~output", Homography, queue_size=1, latch=True)

        # subscriptions:

        # imu
        #self.sub_imu = rospy.Subscriber(
        #    rospy.get_param( "~sub_imu" )
        #    , Something, self.publish, queue_size=1
        #    )

    def publish( self, msg = None ):

        #if msg is not None:
        #   todo: convert IMU data to YPR ( in degrees ), set self.imu_*

        # publish at least once, but don't continue to publish if no subscribers
        if not self.has_published or self.pub.get_num_connections() > 0:

            # compute homography matrix
            H = warpMatrix( 
                rospy.get_param('~radar_img_w') 
                , rospy.get_param('~radar_img_w')
                , rospy.get_param('~yaw') + self.imu_yaw
                , rospy.get_param('~pitch') + self.imu_pitch
                , rospy.get_param('~roll') + self.imu_roll
                , 1.
                , rospy.get_param('~fovy')
                , rospy.get_param('~tx')
                , rospy.get_param('~ty')
                , rospy.get_param('~tz')
                )

            # publish the message
            msg = Homography()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = rospy.get_param("~frame_id")
            msg.child_frame_id = rospy.get_param("~child_frame_id")
            #  warpMatrix returns the radar to rgb homography, but we want to publish the inverse
            msg.values = np.ravel( np.linalg.inv(H) )
            
            self.pub.publish( msg )
            self.has_published = True

if __name__ == "__main__":

    try:
        rospy.init_node(homography_node.__name__)
        n = homography_node()
        n.publish()  # publish one time (with latch), subsequent publishes will be triggered by receipt of IMU data
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
