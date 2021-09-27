#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

from threading import Lock
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
from asv_perception_common import utils

class darknet_node(object):
    """
        Node to perform object detection

        Subscriptions: 
            - ~[0,...,~n_inputs]/input:   [sensor_msgs/CompressedImage] rgb camera image

        Publications:  
            - ~[0,...,~n_inputs]/output:  [asv_perception_common/ClassificationArray]
                        Array of detections.  Resulting ROIs scaled to input image size

            - ~[0,...,~n_inputs]/image:   [sensor_msgs/Image] annotated image (for visualization/debugging)
        
        Parameters:
            - ~n_inputs:    [int, default=1]  number of ~input[0...n] subscriptions and corresponding publications
            - ~classes:     [array[string], default=None]  list of classifier classes to emit.  must be lower case.  If empty, no restriction
            - ~darknet_config_file: [string]  path to darknet config file
            - ~darknet_weights_file: [string]  path to darknet weights file
            - ~darknet_meta_file: [string]  path to darknet meta file
            - ~darknet_thresh: [float, default=0.5]  darknet threshold
            - ~darknet_hier_thresh: [float, default=0.5]  darknet hier_thres
            - ~darknet_nms: [float, default=0.45]  darknet nms
    """

    def __init__(self):

        self.node_name = rospy.get_name()
        self.lock = Lock()

        self.n_inputs = rospy.get_param("~n_inputs", 1 )
        self.classes = rospy.get_param( "~classes", [] )

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

        # publishers array of classification array
        self.pubs = [ rospy.Publisher( '~%d/output' % i, ClassificationArray, queue_size=1 ) for i in range(self.n_inputs) ]

        # image publishers for visualization/debugging
        self.pubs_img = [ rospy.Publisher( '~%d/image' % i, Image, queue_size=1 ) for i in range(self.n_inputs) ]

        # subscriptions array
        self.subs = [ rospy.Subscriber( '~%d/input' % i, CompressedImage, queue_size=1, callback=self.cb_sub, callback_args=i, buff_size=2**24 ) for i in range(self.n_inputs) ]

        print("darknet_node name : {0}".format(self.node_name))
        
    def cb_sub( self, msg, idx ):
        """ perform callback for image message at input index """
        
        # get associated publishers for this input index
        pub = self.pubs[idx]
        pub_img = self.pubs_img[idx]

        # no subscribers, no work
        if pub.get_num_connections() < 1 and pub_img.get_num_connections() < 1:
            return

        rospy.logdebug( 'Processing img with timestamp secs=%d, nsecs=%d', msg.header.stamp.secs, msg.header.stamp.nsecs )

        dets, img = self.detect( msg ) # perform detection

        if pub.get_num_connections() > 0:  # publish detections
            pub.publish( dets )
        
        if pub_img.get_num_connections() > 0:  # publish annotated image
            annotated = self.annotate( img, dets )
            pub_img.publish( annotated )

        
    def detect(self, image_msg):
        """ perform object detection in image message, return ClassificationArray, OpenCV Image """

        # darknet requires rgb image in proper shape.  we need to resize, and then convert resulting bounding boxes to proper shape
        img = utils.convert_ros_msg_to_cv2( image_msg, 'rgb8' )

        orig_shape = img.shape # row, columns, channel

        print("In darknet, orig image shape h:{0}, w:{1}, c:{2}".format(orig_shape[0], orig_shape[1], orig_shape[2]))
        print("Resize shape=h:{0}, w:{1}".format(self.net_img_h, self.net_img_w ))

        # need distortion-free center crop; detector likely requires square image while input is likely widescreen
        img, offsets, scale = utils.resize( img, ( self.net_img_h, self.net_img_w ) ) # do crop/resize

        scale_up = 1./ float(scale) #invert scale to convert from resized --> orig
        offsets = np.abs(offsets // 2) # divide offsets by 2 for center crop.  make positive

        # convert to darknet img format
        #  todo:  use darknet/opencv instead?
        img_data = darknet_array_to_image( img )  #returns tuple

        # returns [(nameTag, dets[j].prob[i], (b.x, b.y, b.w, b.h))]:
        dets = []
        with self.lock:  # darknet apparently not thread-safe, https://github.com/pjreddie/darknet/issues/655
            dets = darknet_detect( self.net, self.meta, img_data[0], thresh=self.darknet_thresh, hier_thresh=self.darknet_hier_thresh, nms=self.darknet_nms )

        # if list of classes is specified, perform filtering on detected classes
        if dets and self.classes:
            dets = filter( lambda det : det[0] and str(det[0]).strip().lower() in self.classes, dets )

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

        return msg, img

    def annotate( self, img, clsMsg ):
        """
        Create the annotated image for visualization/debugging
        """

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

        # create ros msg from img
        msg = utils.convert_cv2_to_ros_msg( img, 'rgb8' )
        msg.header = clsMsg.header # match timestamp
        return msg

if __name__ == "__main__":

    try:
        rospy.init_node(darknet_node.__name__)
        n = darknet_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
