#!/usr/bin/env python
import rospy
import asv_perception_utils as utils
from sensor_msgs.msg import CompressedImage, Image

class correct_rgb_image_node(object):

    def __init__(self):

        self.node_name = rospy.get_name()
        
        # publisher
        self.pub = rospy.Publisher( "~output", Image, queue_size=1)

        # subscriber
        self.sub = rospy.Subscriber( "~input", CompressedImage, self.processImage, queue_size=1 )

    def processImage(self, image_msg):

        # no subscribers, no work
        if self.pub.get_num_connections() == 0:
            return
        
        result = utils.convert_ros_msg_to_cv2( image_msg )

        # do crop/resize
        result = utils.resize_crop( result
            , rospy.get_param("~width")
            , rospy.get_param("~height")
        )
        
        self.pub.publish( utils.convert_cv2_to_ros_msg( result, 'bgr8' ))

if __name__ == "__main__":

    try:
        rospy.init_node( correct_rgb_image_node.__name__ )
        n = correct_rgb_image_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
