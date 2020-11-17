"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

# based on https://gist.github.com/awesomebytes/958a5ef9e63821a28dc05775840c34d9
bridge = CvBridge()

def convert_ros_msg_to_cv2( msg, encoding='passthrough'):
    """ Convert ros message to cv2 image
        Supported encodings for input types:
        - Numpy array:  passthrough
        - sensor_msgs/Image:  see CvBridge docs
        - sensor_msgs/CompressedImage:  passthrough, bgr8, mono8
    """

    global bridge
    if type(msg) == np.ndarray:
        assert( encoding == 'passthrough' )
        return msg
    elif msg._type == 'sensor_msgs/Image':
         return bridge.imgmsg_to_cv2( msg, encoding )
    elif msg._type == 'sensor_msgs/CompressedImage':

        img = cv2.imdecode( np.fromstring( msg.data, np.uint8 ), cv2.IMREAD_COLOR)

        if encoding =='passthrough' or encoding == 'bgr8':  # done
            return img

        # handle encoding change.  assuming img is currently bgr8
        if encoding == 'rgb8':        
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB )
        elif encoding == 'mono8':
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY )
        else:
            raise TypeError("Encoding not implemented: " + encoding )

        return img
    
    raise TypeError("Cannot convert type: " + str(type(msg)))

def convert_cv2_to_ros_msg( cv2_data, image_encoding='passthrough' ):
    """
    Convert from a cv2 image to a ROS Image message.
    Note that encoding is not actually changed, see
    http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
    """
    global bridge
    return bridge.cv2_to_imgmsg(cv2_data, image_encoding)

def compute_resize_params( src, dst, method = 'crop' ):
    """
    Computes crop/pad and scale values to resize between src and dst shapes without distortion
    src:  src shape of (h,w)
    dst:  dst shape of (h,w)
    method:  'crop' or 'pad'
    returns: ( crop/pad (h,w), scale )
    """

    assert src[0] != 0
    assert dst[0] != 0

    if method == 'pad':
        # use crop vals from dst to src, invert
        dims, scale = compute_resize_params( dst, src, 'crop' )
        return dims*-1, 1/scale

    # else, cropping
    if method != 'crop':
        raise NotImplementedError(method)

    # compute aspect ratios
    src_ar = src[1]/float(src[0])
    dst_ar = dst[1]/float(dst[0])
    
    crop = np.array([0,0],dtype=np.int32)

    if not np.isclose([ src_ar ], [ dst_ar ])[0]:  #np float comparison
        if src_ar > dst_ar: #crop width of src
            # crop width = src width - new width
            crop[1] = src[1] - int( dst_ar*src[0] )
        else: # crop height of src
            # crop height = src height - new height
            crop[0] = src[0] - int( src[1]/dst_ar )  

    # compute scaling factor from cropped src to dst
    #  dimension does not matter at this point, since goal is no distortion
    #  dst sz / cropped_src sz
    scale = dst[0] / float( src[0] - crop[0] ) 
        
    return crop*-1, scale

def apply_offsets( img, offsets, method='center' ):
    
    if offsets[0] == offsets[1] == 0:
        return img

    if method != 'center':
        raise NotImplementedError(method)
    
    # compute center offsets
    off_y = offsets[0] // 2
    off_x = offsets[1] // 2
    
    if off_y < 0:  # crop y
        img = img[ (off_y*-1):( img.shape[0] + off_y ) ]
    elif off_y > 0: # pad y
        shapes = [(0,0) for x in range(len(img.shape))] # must account for all dimensions
        shapes[0]=(off_y,off_y)
        img = np.pad( img, shapes, mode='constant' )
        
    if off_x < 0: # crop x
        img = img[ :, (off_x*-1):( img.shape[1] + off_x ) ]
    elif off_x > 0: # pad x
        shapes = [(0,0) for x in range(len(img.shape))] # must account for all dimensions
        shapes[1]=(off_x,off_x)
        img = np.pad( img, shapes, mode='constant' )

    return img

def resize( img, shape, method='crop', interpolation=cv2.INTER_LINEAR ):
    """
    Center pads/crops and scales to desired shape (h,w)
    Returns resized img, offsets (y,x), and scale (float)
    """

    offsets, scale = compute_resize_params( img.shape, shape, method )

    # if crop, apply offsets then scale
    # if pad, scale then apply offsets

    if method=='crop':
        img = apply_offsets( img, offsets ) # crop first
        img = cv2.resize( img, (shape[1],shape[0]), interpolation=interpolation )
    elif method=='pad':
        prepad_shape = shape[:2] - offsets  # resize to prepad shape, then pad
        img = cv2.resize( img, (prepad_shape[1], prepad_shape[0] ), interpolation=interpolation )
        img = apply_offsets( img, offsets )
    else:
        raise NotImplementedError(method)

    return img, offsets, scale