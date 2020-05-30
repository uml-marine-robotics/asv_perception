#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from wasr_net import WASR_net, IMG_SIZE as wasr_img_shape

from sensor_msgs.msg import Image, CompressedImage
import asv_perception_utils as utils

class segmentation_node(object):

    def __init__(self):

        self.node_name = rospy.get_name()

        # wasr pixel value remapping
        self.water_pixel_value = rospy.get_param("~water_pixel_value")
        self.obstacle_pixel_value = rospy.get_param("~obstacle_pixel_value")
        self.sky_pixel_value = rospy.get_param("~sky_pixel_value")

        # init wasr
        self.wasr = WASR_net( rospy.get_param("~model_path") )
        
        # publisher
        self.pub = rospy.Publisher( "~output", Image, queue_size=1)

        # subscribers
        self.sub = rospy.Subscriber( "~input", CompressedImage, self.processImage, queue_size=1 )
        

    def processImage(self, image_msg):

        # no subscribers, no work
        if self.pub.get_num_connections() <= 0:
            return

        img_orig = utils.convert_ros_msg_to_cv2( image_msg, 'bgr8' )
        # orig_h, orig_w = img_orig.shape[:2]

        # resize img to wasr expected shape (512w x 384h)
        # img = cv2.resize( img_orig,( wasr_img_shape[1], wasr_img_shape[0] ), interpolation = cv2.INTER_LINEAR)
        resized = utils.resize_crop( img_orig, wasr_img_shape )
        img = resized[0]

        # run wasr
        img = self.wasr.run_wasr_inference( img )

        # remap wasr classes iaw config
        # obstacles=0, water=1, sky=2
        img = np.where( img==0, self.obstacle_pixel_value, img )  # obstacles 
        img = np.where( img==1, self.water_pixel_value, img )  # water
        img = np.where( img==2, self.sky_pixel_value, img )    # sky

        # keep it at the reduced resolution; subscriber will resize as needed
        msg = utils.convert_cv2_to_ros_msg( img, 'mono8' )
        msg.header = image_msg.header
        self.pub.publish( msg )

if __name__ == "__main__":

    try:
        rospy.init_node(segmentation_node.__name__)
        n = segmentation_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
