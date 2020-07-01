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

        # pixel value remapping for output segmented image
        self.pixel_remap_value = rospy.get_param("~pixel_remap_value")

        # init wasr
        self.wasr = WASR_net( rospy.get_param("~model_path"), rospy.get_param("~per_process_gpu_memory_fraction") )
        
        # publishers
        self.pub_obstacles = rospy.Publisher( "~obstacles", Image, queue_size=1 )
        self.pub_water = rospy.Publisher( "~water", Image, queue_size=1 )
        self.pub_sky = rospy.Publisher( "~sky", Image, queue_size=1 )

        # subscribers
        self.sub = rospy.Subscriber( "~input", CompressedImage, self.cb_img, queue_size=1, buff_size=2**24 )

    # remap pixel class, create ros msg from img
    def remap_pixel_class( self, img, px_class, remap, orig_shape ):

        img_remapped = np.zeros_like(img)
        img_remapped = np.where( img==px_class, remap, img_remapped )
        
        # resize back to original, pad as needed
        img_remapped = utils.resize( img_remapped, orig_shape, 'pad', interpolation=cv2.INTER_NEAREST )[0]
        return utils.convert_cv2_to_ros_msg( img_remapped, 'mono8' )

    def cb_img(self, image_msg):

        # no subscribers, no work
        if self.pub_obstacles.get_num_connections() < 1 and self.pub_water.get_num_connections() < 1 and self.pub_sky.get_num_connections() < 1:
            return

        img = utils.convert_ros_msg_to_cv2( image_msg, 'bgr8' ) # wasr does bgr->rgb

        orig_shape = img.shape

        # resize img to wasr expected shape (512w x 384h)
        img = utils.resize( img, wasr_img_shape )[0] # distortion-free crop/resize

        # run wasr
        img = self.wasr.predict( img )

        # remap wasr classes iaw config
        # wasr classes:  obstacles=0, water=1, sky=2
        if self.pub_obstacles.get_num_connections() > 0:
            msg = self.remap_pixel_class( img, 0, self.pixel_remap_value, orig_shape )
            msg.header = image_msg.header
            self.pub_obstacles.publish( msg )

        if self.pub_water.get_num_connections() > 0:
            msg = self.remap_pixel_class( img, 1, self.pixel_remap_value, orig_shape )
            msg.header = image_msg.header
            self.pub_water.publish( msg )

        if self.pub_sky.get_num_connections() > 0:
            msg = self.remap_pixel_class( img, 2, self.pixel_remap_value, orig_shape )
            msg.header = image_msg.header
            self.pub_sky.publish( msg )

if __name__ == "__main__":

    try:
        rospy.init_node(segmentation_node.__name__)
        n = segmentation_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
