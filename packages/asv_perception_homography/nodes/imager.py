#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from asv_perception_common.NodeLazy import NodeLazy

import numpy as np
import cv2
import math

class pointcloud_imager( NodeLazy ):
    """
      create flattened image from pointcloud for homography

      Publications:
        - output  [sensor_msgs/Image]  output image

      Subscriptions:
        - input   [asv_perception_common/RadarSegment]

      Parameters:
        ~max_range:     [float, default=500]  Maximum radar range to publish, in meters
        ~image_size:    [int, default=max_range*2]   Size of output image width and height (square image)
        ~range_rings:   [int, default=4]   Number of range rings to display
    """

    def __init__(self):

        self.bridge = CvBridge()

        self.max_range = float( rospy.get_param( "~max_range", 500.0 ) )
        self.max_range_squared = self.max_range**2

        # output image size
        self.image_size = rospy.get_param("~image_size", self.max_range*2 )
        self.center_pt = ( int(self.max_range), int(self.max_range) )

        # bgr colors for output
        self.point_color = (0,255,0)
        self.range_color = (0,0,255)
        self.range_label_color = (0,0,255)

        # range ring distances
        self.ranges = [self.max_range]
        for i in range(rospy.get_param("~range_rings", 4) - 1):
            self.ranges.append( int(self.ranges[len(self.ranges)-1]/2.) )

        self.range_thickness = 1
        self.print_ring_labels = False
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.5
        self.text_offset = (-30,+20)  #x,y text offset from range ring

        self.subs = []

        # publisher
        self.pub = self.advertise( '~output', Image, queue_size=1 )
        
    def subscribe( self ):        
        self.unsubscribe()
        self.subs.append( rospy.Subscriber( "~input", PointCloud2, self.cb_cloud, queue_size=1 ) )

    def unsubscribe( self ):
        for sub in self.subs:
            sub.unregister()
        self.subs = []

    def cb_cloud( self, msg ):

        # no subscribers/data, no work
        if self.pub.get_num_connections() < 1:
            return
        
        # holds intensity at each x,y coordinate
        img = np.zeros((int(self.max_range*2.),int(self.max_range*2.),3), np.uint8)

        for pt in point_cloud2.read_points( msg, skip_nans=True ):
            
            # range check
            if ( pt[0]**2 + pt[1]**2 + pt[2]**2 ) > self.max_range_squared:
                continue

            # convert points in cartesian rep-103 (right hand rule) to u,v
            y = int(pt[0]*-1. + self.max_range)
            x = int(pt[1]*-1. + self.max_range)

            # outside max range
            assert x >= 0 and x < img.shape[1]
            assert y >= 0 and y < img.shape[0]

            # place point
            # TODO:  msg may have intensity fields, can scale point value by this intensity
            img[y,x]=self.point_color
        
        # draw rings and corresponding labels (optional) at each range
        for r in self.ranges:
            cv2.circle( img, self.center_pt, int((r/self.max_range) * int(self.max_range)), self.range_color, thickness=self.range_thickness )
            if self.print_ring_labels:
              cv2.putText( img," %.0fM" % r,  
                ( int(self.max_range) + self.text_offset[0], int(self.max_range - r ) + self.text_offset[1]), 
                self.font, self.font_scale, self.range_label_color, 1, cv2.LINE_AA
                )

        # resize to target shape
        img = cv2.resize( img, (self.image_size,self.image_size), interpolation=cv2.INTER_LINEAR )
        
        # publish 
        result = self.bridge.cv2_to_imgmsg( img, 'bgr8')
        result.header = msg.header   
        self.pub.publish( result )

if __name__ == "__main__":

    try:
        rospy.init_node(pointcloud_imager.__name__)
        n = pointcloud_imager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass