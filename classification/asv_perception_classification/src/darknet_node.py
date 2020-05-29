#!/usr/bin/env python
# import os
import rospy
import darknet
import numpy as np
import cv2

from sensor_msgs.msg import Image
from asv_perception_common.msg import Classification, ClassificationArray
import asv_perception_utils as utils

class darknet_node(object):

    def __init__(self):

        self.node_name = rospy.get_name()

        # init darknet
        configPath = rospy.get_param("~darknet_config_file")
        weightPath = rospy.get_param("~darknet_weights_file")
        metaPath = rospy.get_param("~darknet_meta_file")

        self.net = darknet.load_net(str(configPath).encode("ascii"), str(weightPath).encode("ascii"), 0)
        self.meta = darknet.load_meta( str(metaPath).encode("ascii") )

        # get network expected image shape params
        self.net_img_w, self.net_img_h = darknet.get_expected_shape( self.net )

        # publisher
        self.pub = rospy.Publisher( "~output", ClassificationArray, queue_size=1)

        # image publisher for visualization/debugging
        self.pub_img = rospy.Publisher( "~image_raw", Image, queue_size=1)

        # subscribers
        self.sub = rospy.Subscriber( "~input", Image, self.processImage, queue_size=1 )
        

    def processImage(self, image_msg):

        # no subscribers, no work
        if self.pub.get_num_connections() <= 0 and self.pub_img.get_num_connections() <= 0:
            return

        # darknet requires rgb image in proper shape.  we need to resize, and then convert resulting bounding boxes to proper shape
        img_orig = utils.convert_ros_msg_to_cv2(image_msg, 'rgb8' )
        orig_h, orig_w = img_orig.shape[:2]

        img = cv2.resize( img_orig,( self.net_img_w, self.net_img_h ), interpolation = cv2.INTER_LINEAR)

        # def detect(net, meta, im, thresh=.5, hier_thresh=.5, nms=.45, debug= False) -> [(nameTag, dets[j].prob[i], (b.x, b.y, b.w, b.h))]:
        # todo:  parameterize darknet params
        dets = darknet.detect( self.net, self.meta, img )

        # compute scaling params to restore bounding boxes to original img size
        scale_w = orig_w / float( self.net_img_w )
        scale_h = orig_h / float( self.net_img_h )

        msg = ClassificationArray()
        msg.header = image_msg.header #match timestamps
        for det in dets:
            cls = Classification()
            cls.label = det[0]
            cls.probability = det[1]
            roi = det[2]
            
            cls.roi.x_offset = roi[0] * scale_w
            cls.roi.y_offset = roi[1] * scale_h
            w = roi[2]*scale_w
            h = roi[3]*scale_h
            cls.roi.width = np.uint32( w )
            cls.roi.height = np.uint32( h )

            # darknet x,y is center of detection, RegionOfInterest x & y are left- and top-most coords
            cls.roi.x_offset = np.uint32( cls.roi.x_offset - ( w / 2. ) )
            cls.roi.y_offset = np.uint32( cls.roi.y_offset - ( h / 2. ) )

            # cls.roi.do_rectify meaning?
            msg.classifications.append(cls)

        self.pub.publish( msg )

        self.publishImage( img_orig, msg )

    def publishImage( self, img, clsMsg ):
        """
        Publish the annotated image for visualization/debugging
        """
        # no subscribers, no work
        if self.pub_img.get_num_connections() <= 0:
            return

        for cls in clsMsg.classifications:
            (w, h) = (cls.roi.width, cls.roi.height)
            (x, y) = ( cls.roi.x_offset, cls.roi.y_offset )
            
            # draw a bounding box rectangle and label on the image
            color = (255,0,0)
            cv2.rectangle( img, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format( cls.label, cls.probability )
            cv2.putText( img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2 )

        # publish
        msg = utils.convert_cv2_to_ros_msg( img, 'bgr8' )
        msg.header = clsMsg.header # match timestamp
        self.pub_img.publish( msg )


if __name__ == "__main__":

    try:
        rospy.init_node(darknet_node.__name__)
        n = darknet_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
