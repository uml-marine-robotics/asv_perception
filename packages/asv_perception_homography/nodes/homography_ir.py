#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import rospy
import numpy as np
from std_msgs.msg import Header, Empty
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from asv_perception_common.msg import Homography
from asv_perception_common import utils

from asv_perception_homography.calibrate_utils import create_warp_matrix, get_radar_to_world_matrix
from asv_perception_homography.FeedforwardImuController import FeedforwardImuController

class homography_node(object):

    def __init__(self):
        
        self.node_name = rospy.get_name()

        # feed forward imu controller
        self.ctrl = FeedforwardImuController()
        self.last_imu_msg = None

        self.has_published = False
        
        # publish with latch in case of no IMU/testing/etc
        self.pub_ir_radarimg = rospy.Publisher( "~ir_radarimg", Homography, queue_size=1, latch=True)
        self.pub_radarimg_radar = rospy.Publisher( "~radarimg_radar", Homography, queue_size=1, latch=True)
        self.pub_ir_radar = rospy.Publisher( "~ir_radar", Homography, queue_size=1, latch=True)

        # subscriptions:

        # imu
        self.sub_imu = rospy.Subscriber( "~imu", Imu, self.cb_imu, queue_size=1 )

        # refresh notification from calibration tool; useful for visualization updates when rosbag is paused/stopped
        self.sub_refresh = rospy.Subscriber( "~refresh", Empty, self.cb_refresh, queue_size=1 )

        print("homography_node : {0} created.".format(self.node_name))
    
    def cb_refresh( self, msg ):
        self.publish()

    def cb_imu( self, msg ):
        self.last_imu_msg = msg
        self.publish()

    # publish from rgb to other frame
    def publish_homography( self, pub, M, t, frame_id, child_frame_id ):
        msg = Homography()
        msg.header.stamp = t
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.values = np.ravel( M )
        pub.publish( msg )
        self.has_published = True

        print("Homography node name = {0}".format(self.node_name))


    def publish( self ):

        print("IR homography: self.has_published={0}".format(self.has_published))
        print("IR homography: self.pub_ir_radarimg.get_num_connections()={0}".format(self.pub_ir_radarimg.get_num_connections()))
        print("IR homography: self.pub_radarimg_radar.get_num_connections()={0}".format(self.pub_radarimg_radar.get_num_connections()))
        print("IR homography: self.pub_ir_radar.get_num_connections()={0}".format(self.pub_ir_radar.get_num_connections()))

        # check for early exit
        #if self.has_published and self.pub_ir_radarimg.get_num_connections() < 1 and self.pub_radarimg_radar.get_num_connections() < 1 and self.pub_ir_radar.get_num_connections() < 1:
        #    print("XXXX No connections in IR homography XXXX")
        #    return

        # message time
        t = rospy.Time.now()

        # update ff ctrl params, update
        self.ctrl.yaw_alpha = rospy.get_param('~imu_yaw_alpha', 0. )
        self.ctrl.yaw_beta = rospy.get_param('~imu_yaw_beta', 0. )
        self.ctrl.yaw_gamma = rospy.get_param('~imu_yaw_gamma', 0. )

        self.ctrl.pitch_alpha = rospy.get_param('~imu_pitch_alpha', 0. )
        self.ctrl.pitch_beta = rospy.get_param('~imu_pitch_beta', 0. )
        self.ctrl.pitch_gamma = rospy.get_param('~imu_pitch_gamma', 0. )

        self.ctrl.roll_alpha = rospy.get_param('~imu_roll_alpha', 0. )
        self.ctrl.roll_beta = rospy.get_param('~imu_roll_beta', 0. )
        self.ctrl.roll_gamma = rospy.get_param('~imu_roll_gamma', 0. )

        if not self.last_imu_msg is None:
            self.ctrl.update( self.last_imu_msg )

        # rgb to radar
        #  create_warp_matrix computes radar to rgb, we want the inverse
        M_rgb_radar = np.linalg.inv( create_warp_matrix( 
            rospy.get_param('~radar_img_w') 
            , rospy.get_param('~radar_img_w')
            , rospy.get_param('~yaw') - np.degrees( self.ctrl.yaw )
            , rospy.get_param('~pitch') - np.degrees( self.ctrl.pitch )
            , rospy.get_param('~roll') - np.degrees( self.ctrl.roll )
            , 1.
            , rospy.get_param('~fovy')
            , rospy.get_param('~tx')
            , rospy.get_param('~ty')
            , rospy.get_param('~tz')
            ) 
            )

        rgb_frame_id = rospy.get_param("~ir_frame_id")
        radarimg_frame_id = rospy.get_param("~radarimg_frame_id")
        radar_frame_id = rospy.get_param("~radar_frame_id")

        self.publish_homography( self.pub_ir_radarimg, M_rgb_radar, t, rgb_frame_id, radarimg_frame_id )
        print("IR pub_ir_radarimg={0}".format(M_rgb_radar))

        # radar to robot
        #  multiply radar range by 2 to get diameter
        M_radar_robot = get_radar_to_world_matrix( rospy.get_param('~radar_img_w'), 2. * rospy.get_param('~radar_range') )
        self.publish_homography( self.pub_radarimg_radar, M_radar_robot, t, radarimg_frame_id, radar_frame_id )
        print("IR M_radar_robot={0}".format(M_radar_robot))

        # rgb to robot is (radar_to_robot)*(rgb_to_radar)
        M_rgb_robot = np.matmul( M_radar_robot, M_rgb_radar )
        self.publish_homography( self.pub_ir_radar, M_rgb_robot, t, rgb_frame_id, radar_frame_id )
        print("IR pub_ir_radar={0}".format(M_rgb_robot))

if __name__ == "__main__":

    try:
        rospy.init_node(homography_node.__name__)
        n = homography_node()
        n.publish()  # publish one time with latch, subsequent publishes will be triggered by receipt of IMU data
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
