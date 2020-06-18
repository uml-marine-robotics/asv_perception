#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from darknet import (
    detect_image as darknet_detect,
    lib as darknet_lib, 
    load_net as darknet_load_net, 
    load_meta as darknet_load_meta,
    array_to_image as darknet_array_to_image
)
from sensor_msgs.msg import Image, CompressedImage
from asv_perception_common.msg import Classification, ClassificationArray
import asv_perception_utils as utils

class darknet_node(object):

    def __init__(self):

        self.node_name = rospy.get_name()

        # init darknet
        configPath = rospy.get_param("~darknet_config_file")
        weightPath = rospy.get_param("~darknet_weights_file")
        metaPath = rospy.get_param("~darknet_meta_file")
        
        #darknet params; using defaults from darknet.py
        self.darknet_thresh = rospy.get_param("~darknet_thresh", 0.5 )
        self.darknet_hier_thresh = rospy.get_param("~darknet_hier_thresh", 0.5 )
        self.darknet_nms = rospy.get_param("~darknet_nms", 0.45 )

        self.net = darknet_load_net(str(configPath).encode("ascii"), str(weightPath).encode("ascii"), 0)
        self.meta = darknet_load_meta( str(metaPath).encode("ascii") )

        # get network expected image shape params
        self.net_img_w = darknet_lib.network_width( self.net )
        self.net_img_h = darknet_lib.network_height( self.net )

        # publisher of classification array
        self.pub = rospy.Publisher( "~output", ClassificationArray, queue_size=1)

        # image publisher for visualization/debugging
        self.pub_img = rospy.Publisher( "~image", Image, queue_size=1)

        # subscribers
        self.sub = rospy.Subscriber( "~input", CompressedImage, self.processImage, queue_size=1, buff_size=2**24 )
        

    def processImage(self, image_msg):

        # no subscribers, no work
        if self.pub.get_num_connections() <= 0 and self.pub_img.get_num_connections() <= 0:
            return

        rospy.logdebug( 'Processing img with timestamp secs=%d, nsecs=%d', image_msg.header.stamp.secs, image_msg.header.stamp.nsecs )

        # darknet requires rgb image in proper shape.  we need to resize, and then convert resulting bounding boxes to proper shape
        img = utils.convert_ros_msg_to_cv2( image_msg, 'rgb8' )

        orig_shape = img.shape

        # need distortion-free center crop; detector likely requires square image while input is likely widescreen
        img, offsets, scale = utils.resize( img, ( self.net_img_h, self.net_img_w ) ) # do crop/resize

        scale_up = 1./ float(scale) #invert scale to convert from resized --> orig
        offsets = np.abs(offsets // 2) # divide offsets by 2 for center crop.  make positive

        # convert to darknet img format
        #  todo:  use darknet/opencv instead?
        img_data = darknet_array_to_image( img )  #returns tuple

        # returns [(nameTag, dets[j].prob[i], (b.x, b.y, b.w, b.h))]:
        dets = darknet_detect( self.net, self.meta, img_data[0], thresh=self.darknet_thresh, hier_thresh=self.darknet_hier_thresh, nms=self.darknet_nms )

        msg = ClassificationArray()
        msg.header = image_msg.header #match timestamps
        msg.image_width = orig_shape[1]
        msg.image_height = orig_shape[0]

        for det in dets:
            cls = Classification()
            cls.label = det[0]
            cls.probability = det[1]
            roi = det[2]

            # darknet roi:  ( x, y, w, h ), where x and y are the centers of the detection
            #  RegionOfInterest x & y are left- and top-most coords
            cls.roi.width = roi[2]
            cls.roi.height = roi[3]
            cls.roi.x_offset = roi[0] - ( cls.roi.width / 2. )  # convert to left-most x
            cls.roi.y_offset = roi[1] - ( cls.roi.height / 2. ) # convert to top-most y
            
            # scale roi to orig img size
            cls.roi.x_offset *= scale_up
            cls.roi.width *= scale_up
            cls.roi.y_offset *= scale_up
            cls.roi.height *= scale_up
            
            # append crop offset, convert to uint32
            cls.roi.x_offset = np.uint32( cls.roi.x_offset + offsets[1]  )
            cls.roi.y_offset = np.uint32( cls.roi.y_offset + offsets[0] )
            cls.roi.width = np.uint32( cls.roi.width )
            cls.roi.height = np.uint32( cls.roi.height )

            msg.classifications.append(cls)

        self.pub.publish( msg )

        self.publishImage( img, msg )

    def publishImage( self, img, clsMsg ):
        """
        Publish the annotated image for visualization/debugging
        """
        # no subscribers, no work
        if self.pub_img.get_num_connections() <= 0:
            return

        # upscale/pad img back to orig resolution
        #  bounding boxes are already correct for this resolution
        img = utils.resize( img, ( clsMsg.image_height, clsMsg.image_width ), 'pad' )[0]

        # do color conversion rgb --> bgr (optional)
        #img = cv2.cvtColor( img, cv2.COLOR_RGB2BGR )

        for cls in clsMsg.classifications:
            (w, h) = (cls.roi.width, cls.roi.height)
            (x, y) = ( cls.roi.x_offset, cls.roi.y_offset )
            
            # draw a bounding box rectangle and label on the image
            color = (255,0,0)
            cv2.rectangle( img, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format( cls.label, cls.probability )
            cv2.putText( img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2 )

        # publish
        msg = utils.convert_cv2_to_ros_msg( img, 'rgb8' )
        msg.header = clsMsg.header # match timestamp
        self.pub_img.publish( msg )

if __name__ == "__main__":

    try:
        rospy.init_node(darknet_node.__name__)
        n = darknet_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
