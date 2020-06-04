#!/usr/bin/env python
import rospy
import numpy as np

import message_filters
from sensor_msgs.msg import Image, CompressedImage
from asv_perception_common.msg import Classification, ClassificationArray
import asv_perception_utils as utils

def get_class_img( clsArray, class_map ):
    """ 
    Creates an image with pixels that correspond with the class values, iaw class_map
    Highest probability classes are placed `on top` of lower probability classes when overlap exists

    Parameters
    -----------
    clsArray : ClassificationArray message

    class_map : Dict
        dict of class_string: pixel value for classes that will be included in the output

    Returns
    ----------
    numpy image
        shape is determined by contents of clsArray
    """

    # filter classes by class_map
    # sort classes by probability asc
    # create new img map
    # apply each class to new img map, least-->most probably

    # filter incoming classifications by class_map
    # todo:  case sensitivity?
    classes = filter( lambda c : class_map.has_key( c.label ), clsArray.classifications )

    # sort by probability, ascending
    classes.sort( key = lambda c: c.probability )
    
    # create resulting img
    result = np.zeros( ( clsArray.image_height, clsArray.image_width ) , dtype=np.uint8 )

    for c in classes:

        w = c.roi.width
        h = c.roi.height
        x = c.roi.x_offset
        y = c.roi.y_offset
        val = int( class_map[c.label] )

        # apply class value to result
        result[ y:y+h,x:x+w ] = val

    return result

class fusion_node(object):

    def __init__(self):
        
        self.node_name = rospy.get_name()

        # pixel map
        self.pixel_map = rospy.get_param("~pixel_map", dict() )
        rospy.logdebug("Loaded pixel_map dictionary with %d entries", len(self.pixel_map) )

        # get obstacle pixel value for segmentation map
        self.segmentation_obstacle_pixel_value = rospy.get_param( "~segmentation_obstacle_pixel_value", 128 )
        
        # publisher
        self.pub = rospy.Publisher( "~output", Image, queue_size=1 )

        # subscriptions:
        # todo:  lazy subscriptions; segmentation & classification are expensive operations 
        #   and we'll be calling them whether we have a subscriber or not
        self.sub_seg = message_filters.Subscriber( "~segmentation", Image, queue_size=1, buff_size=2**24 )
        self.sub_cls = message_filters.Subscriber( "~classification", ClassificationArray, queue_size=1 )

        # time synchronizer b/w segmentation and classification
        # any downsides to using ApproximateTimeSynchronizer over TimeSynchronizer, just in case?
        self.ts = message_filters.ApproximateTimeSynchronizer( [self.sub_seg,self.sub_cls], 10, 0.5 )
        self.ts.registerCallback(self.callback)

    # receive callback from timesynchronizer
    def callback( self, segMsg, clsMsg ):
        
        if self.pub.get_num_connections() <= 0:
            return

        # get segmented image, filter out non-obstacle pixels
        segImg = utils.convert_ros_msg_to_cv2( segMsg )
        segImg = np.where( segImg == self.segmentation_obstacle_pixel_value, self.segmentation_obstacle_pixel_value, 0 ).astype(np.uint8)

        # create image from classification data;
        #   color pixels according to pixel_map, highest probability on top
        clsImg = get_class_img( clsMsg, self.pixel_map )

        # input image sizes should match
        assert segImg.shape[:2] == clsImg.shape[:2]

        # combine class map with obstacle map for mask
        #  this refines the classifier bounding boxes to actual obstacle shapes
        mask = np.bitwise_and( clsImg > 0, segImg > 0 )

        # lastly, combine the segmented obstacle map with the refined cls map
        #  class map "on top"
        img = np.where( mask > 0, clsImg, segImg )

        # publish
        msg = utils.convert_cv2_to_ros_msg( img.astype(np.uint8), 'mono8' )
        msg.header = segMsg.header # match timestamp
        self.pub.publish( msg )

if __name__ == "__main__":

    try:
        rospy.init_node(fusion_node.__name__)
        n = fusion_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
