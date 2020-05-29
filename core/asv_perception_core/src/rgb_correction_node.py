#!/usr/bin/env python
import rospy
import asv_perception_utils as utils
from sensor_msgs.msg import CompressedImage, Image

class correct_rgb_image_node(object):

    def __init__(self):

        self.node_name = rospy.get_name()

        # get params
        self.resize_w = rospy.get_param("~width")
        self.resize_h = rospy.get_param("~height")
        
        # publisher
        self.pub = rospy.Publisher( "~output", Image, queue_size=1)

        # subscriber
        self.sub = rospy.Subscriber( "~input", CompressedImage, self.processImage, queue_size=1 )

    def processImage(self, image_msg):

        # no subscribers, no work
        if self.pub.get_num_connections() == 0:
            return
        
        img = utils.convert_ros_msg_to_cv2( image_msg )

        # do crop/resize
        img = utils.resize_crop( 
            img
            , self.resize_w
            , self.resize_h
        )

        msg = utils.convert_cv2_to_ros_msg( img, 'bgr8' )
        msg.header = image_msg.header
        
        self.pub.publish( msg )

if __name__ == "__main__":

    try:
        rospy.init_node( correct_rgb_image_node.__name__ )
        n = correct_rgb_image_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
