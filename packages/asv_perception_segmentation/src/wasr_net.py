#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

""" 
Run WASR inference on a single image, return the segmented image mask
Based on wasr_inference_noimu_general.py at https://github.com/bborja/wasr_network (Apache-2 License)
"""

import tensorflow as tf
import numpy as np

#from wasr_models import wasr_NOIMU2, ImageReader, decode_labels, prepare_label
from wasr_models import wasr_NOIMU2

# COLOR MEANS OF IMAGES FROM MODDv1 DATASET
IMG_MEAN = np.array((148.8430, 171.0260, 162.4082), dtype=np.float32)

# Number of classes
NUM_CLASSES = 7

# Input image size. Our network expects images of resolution 512x384
IMG_SIZE = [384, 512]

""" WASR network object for inference"""
class WASR_net(object):

    def __init__( self, weights_path, image_size=None, image_mean=None, num_classes=None, per_process_gpu_memory_fraction = 0 ):

        # Create network
        if (image_size is None):
            self.img_input = tf.placeholder(dtype=tf.uint8, shape=(IMG_SIZE[0], IMG_SIZE[1], 3))
        else:
            self.img_input = tf.placeholder(dtype=tf.uint8, shape=(image_size[0], image_size[1], 3))

        # Convert from opencv BGR to tensorflow's RGB format
        img_b, img_g, img_r = tf.split(axis=2, num_or_size_splits=3, value=self.img_input)

        # Join and subtract means
        img = tf.cast(tf.concat(axis=2, values=[img_r, img_g, img_b]), dtype=tf.float32)

        if (image_mean is None):
            img -= IMG_MEAN
        else:
            img -= image_mean

        # Expand first dimension
        #img = tf.expand_dims(img, dim=0) # tf 1.2
        img = tf.expand_dims(img, axis=0)

        with tf.variable_scope('', reuse=False):
            if (num_classes is None):
                net = wasr_NOIMU2({'data': img}, is_training=False, num_classes=NUM_CLASSES)
            else:
                net = wasr_NOIMU2({'data': img}, is_training=False, num_classes=num_classes)

        # Which variables to load...
        restore_var = tf.global_variables()

        # Predictions (last layer of the decoder)
        raw_output = net.layers['fc1_voc12']

        # Upsample image to the original resolution
        raw_output = tf.image.resize_bilinear(raw_output, tf.shape(img)[1:3, ])
        #raw_output = tf.argmax(raw_output, dimension=3) # tf 1.2
        #pred = tf.expand_dims(raw_output, dim=3) # tf 1.2
        raw_output = tf.argmax(raw_output, axis=3)
        self.pred = tf.expand_dims(raw_output, axis=3)

        # Set up TF session and initialize variables.
        config = tf.ConfigProto()

        # limit gpu,if specified
        if per_process_gpu_memory_fraction > 0:
            config.gpu_options.per_process_gpu_memory_fraction=per_process_gpu_memory_fraction
        else:
            config.gpu_options.allow_growth = True

        self.sess = tf.Session(config=config)
        init = tf.global_variables_initializer()

        self.sess.run(init)

        # Load weights
        self.loader = tf.train.Saver(var_list=restore_var)
        self.loader.restore( self.sess, weights_path )

    def predict( self, img_in ):

        # Run inference
        preds = self.sess.run( self.pred, feed_dict={self.img_input: img_in})
        
        # Shailesh: use this code at call-site so the preds remains tensor
        # and a color image can be obtained.
        # just convert prediction mask to uint8, return it in 2D
        # result = np.squeeze(preds[0]).astype('uint8')

        return preds