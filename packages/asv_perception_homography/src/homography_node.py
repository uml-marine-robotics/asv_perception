#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Header
from asv_perception_common.msg import Homography
import asv_perception_utils as utils

from calibrate import warpMatrix, get_radar_to_world_matrix

class homography_node(object):

    def __init__(self):
        
        self.node_name = rospy.get_name()
        
        # YPR in degrees
        self.imu_yaw = 0.   
        self.imu_pitch = 0.
        self.imu_roll = 0.

        self.has_published = False
        
        # publish with latch in case of no IMU/testing/etc
        self.pub_rgb_radar = rospy.Publisher( "~rgb_radar", Homography, queue_size=1, latch=True)
        self.pub_radar_world = rospy.Publisher( "~radar_world", Homography, queue_size=1, latch=True)
        self.pub_rgb_world = rospy.Publisher( "~rgb_world", Homography, queue_size=1, latch=True)

        # subscriptions:

        # imu
        #self.sub_imu = rospy.Subscriber(
        #    rospy.get_param( "~sub_imu" )
        #    , Something, self.publish, queue_size=1
        #    )

    def publishHomography( self, pub, M, t ):
        msg = Homography()
        msg.header.stamp = t
        msg.header.frame_id = rospy.get_param("~frame_id")
        msg.child_frame_id = rospy.get_param("~child_frame_id")
        msg.values = np.ravel( M )
        pub.publish( msg )
        self.has_published = True


    def publish( self, msg = None ):

        # check for early exit
        if self.has_published and self.pub_rgb_radar.get_num_connections() < 1 and self.pub_radar_world.get_num_connections() < 1 and self.pub_rgb_world.get_num_connections() < 1:
            return

        #if msg is not None:
        #   todo: convert IMU data to YPR ( in degrees ), set self.imu_*

        # message time
        t = rospy.Time.now()

        # rgb to radar
        #  warpMatrix computes radar to rgb, we want the inverse
        M_rgb_radar = np.linalg.inv( warpMatrix( 
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
            )

        self.publishHomography( self.pub_rgb_radar, M_rgb_radar, t )

        # radar to world
        M_radar_world = get_radar_to_world_matrix( rospy.get_param('~radar_img_w'), rospy.get_param('~radar_world_units') )
        self.publishHomography( self.pub_radar_world, M_radar_world, t )

        # rgb to world is (radar_to_world)*(rgb_to_radar)
        M_rgb_world = np.matmul( M_radar_world, M_rgb_radar )
        self.publishHomography( self.pub_rgb_world, M_rgb_world, t )

if __name__ == "__main__":

    try:
        rospy.init_node(homography_node.__name__)
        n = homography_node()
        n.publish()  # publish one time (with latch), subsequent publishes will be triggered by receipt of IMU data
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
