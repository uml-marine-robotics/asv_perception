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
import scipy.ndimage
from wasr_net import WASR_net, IMG_SIZE as wasr_img_shape
from wasr_models.utils import decode_labels

from sensor_msgs.msg import Image, CompressedImage
from asv_perception_common import utils
from asv_perception_common.msg import Classification, ClassificationArray

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
        self.n_inputs_ir = rospy.get_param( "~n_inputs_ir", 1 )
        self.ir_image_size = rospy.get_param("~ir_image_size") # [h, w]
        self.ir_image_mean = rospy.get_param("~ir_image_mean")
        self.ir_num_classes = rospy.get_param("~ir_num_classes")

        # pixel value remapping for output segmented image
        self.pixel_remap_value = rospy.get_param("~pixel_remap_value")

        # init wasr
        # self.wasr = WASR_net( rospy.get_param("~model_path"), rospy.get_param("~per_process_gpu_memory_fraction", 0 ) )
        self.wasr_ir = WASR_net (rospy.get_param("~ir_model_path"), self.ir_image_size, 
                                 self.ir_image_mean, self.ir_num_classes, rospy.get_param("~per_process_gpu_memory_fraction", 0 ))


        # publishers
        self.pubs_obstacles = [ rospy.Publisher( '~%d/obstacles' % i, Image, queue_size=1 ) for i in range(self.n_inputs_ir) ]
        self.pubs_water = [ rospy.Publisher( '~%d/water' % i, Image, queue_size=1 ) for i in range(self.n_inputs_ir) ]
        self.pubs_sky = [ rospy.Publisher( '~%d/sky' % i, Image, queue_size=1 ) for i in range(self.n_inputs_ir) ]
        self.pubs_total_segmentation = [ rospy.Publisher( '~%d/total_IR_segmentation' % i, Image, queue_size=1 ) for i in range(self.n_inputs_ir) ]
        # publishers array of classification array
        self.pubs_output_classificationArray = [ rospy.Publisher( '~%d/output' % i, ClassificationArray, queue_size=1 ) for i in range(self.n_inputs_ir) ]


        # subscriptions array
        self.subs = [ rospy.Subscriber( '~%d/input_ir' % i, Image, callback=self.cb_sub_ir, callback_args=i, queue_size=1, buff_size=2**24 ) for i in range(self.n_inputs_ir) ]
        self.counter = 1

        self.SKY_ID = 0
        self.WATER_ID = 1
        self.BRIDGE_ID = 2
        self.OBSTACLE_ID = 3
        self.LIVING_OBSTACLE_ID = 4
        self.BACKGROUND_ID = 5
        self.SELF_ID = 6

        self.classificationsArray = ClassificationArray()
        self.orig_image_width = 0
        self.orig_image_height = 0


    def cb_sub_ir( self, msg, idx ):
        """ handle callback for subscription to ir images at index """    
        # get associated publishers for this input index
        # The IR publisher is after the optical publishers. So increase the index by self.n_inputs
        pub_obs = self.pubs_obstacles[idx]
        pub_water = self.pubs_water[ idx]
        pub_sky = self.pubs_sky[idx]  
        pub_total_segmentation = self.pubs_total_segmentation[idx]
        pub_output_classificationArray = self.pubs_output_classificationArray[idx]

        print("Inside IR segmentation cb: IMG_SIZE[0] = {0}, IMG_SIZE[1] = {1}".format(self.ir_image_size[0], self.ir_image_size[1]))

        # convert ros msg to cv img. The IR image is a grayscale image, it needs to be converted
        # to RGB for wasr to consume.
        img = utils.convert_ros_msg_to_cv2( msg) 
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB) # wasr does bgr->rgb
        orig_shape = img.shape

        print("Inside IR segmentation cb: Original image shape={0}".format(orig_shape))
        print("Inside IR segmentation cb: Specified IR image size = {0}".format(self.ir_image_size))
        # resize img to IR expected size
        img, offsets, scale = utils.resize( img, self.ir_image_size )
        print("Inside IR segmentation cb: IR image size after resize = {0}".format(img.shape))
        print("offsets={0}, scale={1}".format(offsets, scale))

        # run segmentation.  tf.run apparently thread safe
        img_Tensor = self.wasr_ir.predict( img )

        # Extract prediction mask
        print("Inside IR segmentation cb: Shape of predicted images = {0}".format(img_Tensor.shape))
        
        #outputFileName = 'mask_%03d.png' % self.counter
        #self.counter = self.counter + 1
        #cv2.imwrite(outputFileName, msk[0])

        # just convert prediction mask to uint8, return it in 2D
        img_2D = np.squeeze(img_Tensor).astype('uint8')

        # no subscribers, no work
        #if pub_obs.get_num_connections() < 1 and pub_water.get_num_connections() < 1 and pub_sky.get_num_connections() < 1:
        #    print("Segmentation IR: No connections to any (obstacle/water/sky) subscribers")
        #    return

        # remap wasr classes iaw config
        # wasr classes:  obstacles=0, water=1, sky=2
        # With 7 classes, self.OBSTACLE_ID and self.LIVING_OBSTACLE_ID
        outputMsg = ClassificationArray()
        outputMsg.header = msg.header #match timestamps
        # Increase the size of IR image (512(h),640(w)) to match RGB image resolution (1024(h), 1280(w))
        outputMsg.image_width = 2*orig_shape[1]
        outputMsg.image_height = 2*orig_shape[0]
        self.orig_image_width = outputMsg.image_width
        self.orig_image_height = outputMsg.image_height

        scale_up = 1./ float(scale) #invert scale to convert from resized --> orig
        scale_up = scale_up * 2.0 # Increase the size of IR image (512(h),640(w)) to match RGB image resolution (1024(h), 1280(w))

        if pub_obs.get_num_connections() > 0:
            # self.LIVING_OBSTACLE_ID
            resultant_image = self.remap_pixel_class_for_obstacles( img_2D, [self.OBSTACLE_ID], self.pixel_remap_value, orig_shape )
            # morphological operators to get rid of very small spurious regions
            resultant_image = cv2.erode(resultant_image, None, iterations=3)
            resultant_image = cv2.dilate(resultant_image, None, iterations=5)
            print("Shape of resultant image={0}".format(resultant_image.shape))

            output_img = utils.convert_cv2_to_ros_msg(resultant_image)
            blobs, num_blobs = scipy.ndimage.label(resultant_image)
            data_slices = scipy.ndimage.find_objects(blobs)
            print("Inside IR segmentation cb: no. of data slices = {0}".format(len(data_slices)))
            for aSlice in data_slices:
                dy, dx = aSlice[:2]
                width = dx.stop+1-dx.start
                height = dy.stop+1-dy.start
                cls = Classification()
                cls.label = "obstacle"
                cls.probability = 0.9 # hard-code for time being
                roi = (dx.start, dy.start, width, height)
                
                # region of interest given by scipy.ndimage.find_objects : dx.start, dy.start represent top left corner
                # of a region
                cls.roi.width = roi[2]+10 # hard-code for time being
                cls.roi.height = roi[3]+10 # hard-code for time being
                cls.roi.x_offset = roi[0]
                cls.roi.y_offset = roi[1]

                # scale roi to orig img size : This is not needed because resultant_image
                # is already scaled up to original image size.
                #cls.roi.x_offset *= scale_up
                #cls.roi.width *= scale_up
                #cls.roi.y_offset *= scale_up
                #cls.roi.height *= scale_up

                # convert to uint32
                cls.roi.x_offset = np.uint32(cls.roi.x_offset)
                cls.roi.y_offset = np.uint32(cls.roi.y_offset)
                cls.roi.width = np.uint32(cls.roi.width)
                cls.roi.height = np.uint32(cls.roi.height)

                print("IR cls.roi.x_offset={0}, cls.roi.y_offset={1}, cls.roi.width={2}, cls.roi.height={3}".format(cls.roi.x_offset,
                cls.roi.y_offset, cls.roi.width, cls.roi.height))

                self.classificationsArray.classifications.append(cls)
                outputMsg.classifications.append(cls)
            
            output_img.header = msg.header
            pub_obs.publish(output_img)
            pub_output_classificationArray.publish( outputMsg )
            print("Inside IR segmentation cb: classificationArray published")

        if pub_water.get_num_connections() > 0:
            out = self.remap_pixel_class( img_2D, self.WATER_ID, self.pixel_remap_value, orig_shape )
            out.header = msg.header
            pub_water.publish( out )

        if pub_sky.get_num_connections() > 0:
            out = self.remap_pixel_class( img_2D, self.SKY_ID, self.pixel_remap_value, orig_shape )
            out.header = msg.header
            pub_sky.publish( out )

        if pub_total_segmentation.get_num_connections() > 0:
            colored_mask = decode_labels(img_Tensor, num_classes=7)
            print("Shape of colored mask={0}".format(colored_mask[0]))
            print("type of colored mask = {0}".format(type(colored_mask[0])))
            #annotated_image = self.annotate(colored_mask, self.classifications)
            out = utils.convert_cv2_to_ros_msg( colored_mask[0])
            out.header = msg.header
            pub_total_segmentation.publish(out)

    def annotate(self, segmentedImage, classifications):
        """
        Create the annotated segmentation image for visualization/debugging
        """
        #annotated_img = utils.resize(segmentedImage, ( self.orig_image_height, self.orig_image_width ), 'pad' )[0]
        annotated_img = segmentedImage

        for cls in classifications:
            (w, h) = (cls.roi.width, cls.roi.height)
            (x, y) = ( cls.roi.x_offset, cls.roi.y_offset )
            
            # draw a bounding box rectangle and label on the image
            color = (255,0,0)
            cv2.rectangle( annotated_img, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format( "Obstacle", cls.probability )
            cv2.putText( annotated_img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2 )
        
        return annotated_img

    def remap_pixel_class_for_obstacles(self, img, px_classes, remap, orig_shape):
        # represents obstacles
        obstacleID1 = px_classes[0]
        img_remapped1 = np.zeros_like(img)
        img_remapped1 = np.where( img==obstacleID1, remap, img_remapped1 )
        resultant_image = img_remapped1

        # represents living obstacles
        if (len(px_classes) > 1):
            obstacleID2 = px_classes[1]
            img_remapped2 = np.zeros_like(img)
            img_remapped2 = np.where( img==obstacleID2, remap, img_remapped2 )
            resultant_image = img_remapped1 + img_remapped2

        # resize back to original, pad as needed
        # Increase the size of IR image (512(h),640(w)) to match RGB image resolution (1024(h), 1280(w))
        new_shape = (2*orig_shape[0], 2*orig_shape[1], 3)
        
        resultant_image = utils.resize( resultant_image, new_shape, 'pad', interpolation=cv2.INTER_NEAREST )[0]
        print("type of remapped image = {0}".format(type(resultant_image)))
        return resultant_image

    # remap pixel value, resize, return ros msg
    def remap_pixel_class( self, img, px_class, remap, orig_shape ):

        # @todo Try to generate color image here. This is not urgent but nice to have.
        img_remapped = np.zeros_like(img)
        img_remapped = np.where( img==px_class, remap, img_remapped )
        
        # resize back to original, pad as needed
        img_remapped = utils.resize( img_remapped, orig_shape, 'pad', interpolation=cv2.INTER_NEAREST )[0]
        
        #return utils.convert_cv2_to_ros_msg( img_remapped, 'mono8' )
        print("type of remapped image = {0}".format(type(img_remapped)))
        return utils.convert_cv2_to_ros_msg( img_remapped)

if __name__ == "__main__":

    try:
        rospy.init_node(segmentation_node.__name__)
        n = segmentation_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
