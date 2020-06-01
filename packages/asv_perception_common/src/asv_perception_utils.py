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

def resize_crop( img, shape ):
    """
    Center crops (if needed) and then resizes to desired shape (h,w) without distortion
    Returns tuple of resized img, (offset_x, offset_y), (scale_x, scale_y)
    - (offset_x, offset_y) is the left- and top-most point at which the crop operation ended
    - (scale_x, scale_y) is the scaling multiplier from the original image to the output image
    Based on https://stackoverflow.com/a/4744625/882436
    """

    h,w = img.shape[:2]
    ar = w/float(h)  # input aspect ratio

    target_h, target_w = shape[:2]
    target_ar = target_w/float(target_h)  # target aspect ratio

    offset_x = 0
    offset_y = 0

    if np.isclose([ ar ], [ target_ar ])[0]:  #np float comparison
        result = img  # no cropping necessary
    else:  # crop needed
        if ar > target_ar: #crop left&right
            new_w = int(target_ar*h)
            offset_x = ( w - new_w ) / 2
        else: # crop top&bottom
            new_h = int(w/target_ar)
            offset_y = (h-new_h) / 2

        #do crop
        result = img[ offset_y:(h-offset_y), offset_x:(w-offset_x) ]

    # compute scaling factors from cropped to resized
    #  target sz / cropped sz
    scale = ( shape[1] / float(result.shape[1]), shape[0] / float(result.shape[0]) )

    # do resize, return vals
    return ( cv2.resize(result, (target_w,target_h)), (offset_x, offset_y), scale )
