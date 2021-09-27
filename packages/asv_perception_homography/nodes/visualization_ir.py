#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, Imu
from tf.transformations import euler_from_quaternion

from asv_perception_common.msg import Homography
from asv_perception_common import utils
from asv_perception_common.NodeLazy import NodeLazy
from asv_perception_homography.calibrate_utils import create_unified_image

class homography_visualization( NodeLazy ):

    def __init__(self):

        self.node_name = rospy.get_name()
        self.ir_image_size = rospy.get_param('~ir_image_size')
        
        self.radar_img = None
        self.ir_img = None
        self.homography = None
        self.imu = None

        self.pub_header = None

        self.subs = []

        # publisher
        self.pub = self.advertise( '~image', Image, queue_size=1 )
        print("homography_visualization={0} created".format(self.node_name))

    def subscribe( self ):

        print("My {0} subscribe method is called".format(self.node_name))
        
        self.unsubscribe()

        # subscribers; approximate time sync seems to fail when restarting a rosbag; just use latest of each msg

        # ir
        self.subs.append( rospy.Subscriber( "~ir", Image, self.cb_ir, queue_size=1, buff_size=2**24 ) )

        # radar image
        self.subs.append( rospy.Subscriber( "~radarimg", Image, self.cb_radar, queue_size=1, buff_size=2**24 ) )

        # homography matrix
        self.subs.append( rospy.Subscriber( "~ir_radarimg", Homography, self.cb_homography, queue_size=1 ) )

        # imu
        self.subs.append( rospy.Subscriber( "~imu", Imu, self.cb_imu, queue_size=1 ) )

        # For debug purpose only:
        self.subs.append( rospy.Subscriber( "~radarimg_radar", Imu, self.cb_radarimg_radar, queue_size=1 ) )
        self.subs.append( rospy.Subscriber( "~ir_radar", Imu, self.cb_ir_radar, queue_size=1 ) )

    def unsubscribe( self ):

        for sub in self.subs:
            sub.unregister()
        self.subs = []

    def cb_radarimg_radar(self, msg):
        print("msg={0}".format(msg.header)) 
        print("visualization_ir : radarimg_radar received")

    def cb_ir_radar(self, msg):
        print("msg={0}".format(msg.header))
        print("visualization_ir : ir_radar received")

    def cb_imu( self, msg ):
        self.imu = msg
        self.publish()

    def cb_ir(self, msg):
        self.ir_img = utils.convert_ros_msg_to_cv2( msg ) 
        self.ir_img = cv2.cvtColor(self.ir_img, cv2.COLOR_GRAY2RGB)
        # Resize to RGB image size so that calibration tool can show IR image properly.
        # 1024 (H), 1280 (W)
        #self.ir_img = utils.resize(self.ir_img, self.ir_image_size )[0]
        self.ir_img = utils.resize(self.ir_img, [1024,1280] )[0]
        self.pub_header = msg.header
        self.publish()
    
    def cb_radar(self, msg ):
        self.radar_img = utils.convert_ros_msg_to_cv2( msg )
        self.publish()

    def cb_homography( self, msg ):
        # convert float[9] to numpy 3x3
        #   then invert the homography, since we want radar --> rgb for image creation
        self.homography = np.linalg.inv( np.array( msg.values ).reshape((3,3)) )
        self.publish()

    def publish(self):

        print("Node name of ir homography visualization={0}".format(self.node_name))
        # no subscribers/data, no work
        if self.pub.get_num_connections() < 1 or self.radar_img is None or self.ir_img is None or self.homography is None:
            print("Homography/visualization_ir: No subscribers")
            return

        print("My {0} publish method is called".format(self.node_name))

        # using rgb msg header
        assert self.pub_header is not None

        irH, irW = self.ir_img.shape[:2]
        radarH, radarW    = self.radar_img.shape[:2]
        print("Homography visualization : irH={0}, irW={1}, radarH={2}, radarW={3}".format(irH, irW, radarH, radarW))
        img = create_unified_image( self.ir_img, self.radar_img, self.homography )

        print("Homography visualization: success with create_unified_image")

        # print some text on the resulting image
        print_text = lambda text, position : cv2.putText( img, text, position, cv2.FONT_HERSHEY_SIMPLEX, 1., (0,255,0), 2 )

        # convert imu orientation to euler angles, display
        if self.imu is not None:
            # imu.orientation is a normalized quaternion.  euler_from_quaternion returns radians
            rpy = euler_from_quaternion( [self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w] )
            rpy = np.degrees( rpy )
            print_text( "{}: {:.2f}".format( "Roll (deg)", rpy[0] ), ( 5, 50 ) )
            print_text( "{}: {:.2f}".format( "Pitch (deg)", rpy[1] ), ( 5, 100 ) )
            print_text( "{}: {:.2f}".format( "Yaw (deg)", rpy[2] ), ( 5, 150 ) )

        msg = utils.convert_cv2_to_ros_msg( img, 'bgr8' )
        msg.header = self.pub_header # match timestamp
        self.pub.publish( msg )

if __name__ == "__main__":

    try:
        rospy.init_node(homography_visualization.__name__)
        n = homography_visualization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
