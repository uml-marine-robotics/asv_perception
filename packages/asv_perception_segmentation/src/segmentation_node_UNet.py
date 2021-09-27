#!/usr/bin/env python

"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import rospy
import numpy as np
import cv2
#from wasr_net import WASR_net, IMG_SIZE as wasr_img_shape
from UNet_model import UNet_model

from sensor_msgs.msg import Image, CompressedImage
from asv_perception_common import utils

class segmentation_node(object):
    """
    Node to perform image segmentation

    Subscriptions: 
        - ~[0,...,~n_inputs]/input:   [sensor_msgs/CompressedImage] rgb camera image

    Publications:  
        - ~[0,...,~n_inputs]/obstacles:   [sensor_msgs/Image] segmented obstacle image, mono8/grayscale
        - ~[0,...,~n_inputs]/water:       [sensor_msgs/Image] segmented water image, mono8/grayscale
        - ~[0,...,~n_inputs]/sky:         [sensor_msgs/Image] segmented sky image, mono8/grayscale
    
    Parameters:
        - ~n_inputs:    [int, default=1]  number of ~[0...n]/input subscriptions and corresponding publications
        - ~per_process_gpu_memory_fraction: [float, 0.0-1.0, default=0] gpu memory limit % of available; wasr requires at least ~1.2GB.  If zero, allows growth as needed
        - ~pixel_remap_value:  [int, 0-255]  segmentation class remapping values
        - ~model_path:  [string]  path to segmentation model file
    """

    def __init__(self):

        self.node_name = rospy.get_name()
        self.n_inputs = rospy.get_param( "~n_inputs", 1)

        # pixel value remapping for output segmented image
        self.pixel_remap_value = rospy.get_param("~pixel_remap_value")

        # init UNet
        #self.wasr = WASR_net( rospy.get_param("~model_path"), rospy.get_param("~per_process_gpu_memory_fraction", 0 ) )
        self.UNet = UNet_model( rospy.get_param("~model_path"), rospy.get_param("~image_size") )

        # publishers
        self.pubs_obstacles = [ rospy.Publisher( '~%d/obstacles' % i, Image, queue_size=1 ) for i in range(self.num_input_images) ]
        self.pubs_water = [ rospy.Publisher( '~%d/water' % i, Image, queue_size=1 ) for i in range(self.num_input_images) ]
        self.pubs_sky = [ rospy.Publisher( '~%d/sky' % i, Image, queue_size=1 ) for i in range(self.num_input_images) ]

        # subscriptions array
        self.subs = [ rospy.Subscriber( '~%d/input' % i, CompressedImage, callback=self.cb_sub, callback_args=i, queue_size=1, buff_size=2**24 ) for i in range(self.n_inputs) ]

        print("Inside UNet segmentation initialization: ok")

    def cb_sub( self, msg, idx ):
        """ handle callback for subscription at index """

        # get associated publishers for this input index
        pub_obs = self.pubs_obstacles[idx]
        pub_water = self.pubs_water[idx]
        pub_sky = self.pubs_sky[idx]

        # no subscribers, no work
        if pub_obs.get_num_connections() < 1 and pub_water.get_num_connections() < 1 and pub_sky.get_num_connections() < 1:
            print("No connections to any (obstacle/water/sky) publishers")
            return

        # convert ros msg to cv img
        img = utils.convert_ros_msg_to_cv2( msg, 'bgr8' ) # wasr does bgr->rgb
        orig_shape = img.shape

        # resize img to UNet expected shape (320w x 256h)
        img = utils.resize( img, rospy.get_param("~image_size"))[0]

        # run segmentation.  tf.run apparently thread safe
        img = self.UNet_model.predict( img )
        
        # remap wasr classes iaw config
        # wasr classes:  obstacles=0, water=1, sky=2
        OBSTACLE_ID = 0
        WATER_ID = 1
        SKY_ID = 2
        if pub_obs.get_num_connections() > 0:
            out = self.remap_pixel_class( img, OBSTACLE_ID, self.pixel_remap_value, orig_shape )
            out.header = msg.header
            pub_obs.publish( out )

        if pub_water.get_num_connections() > 0:
            out = self.remap_pixel_class( img, WATER_ID, self.pixel_remap_value, orig_shape )
            out.header = msg.header
            pub_water.publish( out )

        if pub_sky.get_num_connections() > 0:
            out = self.remap_pixel_class( img, SKY_ID, self.pixel_remap_value, orig_shape )
            out.header = msg.header
            pub_sky.publish( out )

    # remap pixel value, resize, return ros msg
    def remap_pixel_class( self, img, px_class, remap, orig_shape ):

        img_remapped = np.zeros_like(img)
        img_remapped = np.where( img==px_class, remap, img_remapped )
        
        # resize back to original, pad as needed
        img_remapped = utils.resize( img_remapped, orig_shape, 'pad', interpolation=cv2.INTER_NEAREST )[0]
        return utils.convert_cv2_to_ros_msg( img_remapped, 'mono8' )

if __name__ == "__main__":

    try:
        rospy.init_node(segmentation_node.__name__)
        n = segmentation_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
