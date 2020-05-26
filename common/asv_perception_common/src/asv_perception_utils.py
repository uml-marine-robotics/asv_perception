from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
#from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_msgs.msg import String
import json

import roslib
roslib.load_manifest('asv_perception_common')

# based on https://gist.github.com/awesomebytes/958a5ef9e63821a28dc05775840c34d9
bridge = CvBridge()
def convert_ros_msg_to_cv2( msg, encoding='passthrough'):

    global bridge
    if type(msg) == np.ndarray:
        return msg
    elif msg._type == 'sensor_msgs/Image':
         return bridge.imgmsg_to_cv2( msg, encoding )
    elif msg._type == 'sensor_msgs/CompressedImage':
        assert( encoding =='passthrough' )# todo:  handle encoding change
        np_arr = np.fromstring( msg.data, np.uint8 )
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    raise TypeError("Cannot convert type: " + str(type(msg)))

def convert_cv2_to_ros_msg( cv2_data, image_encoding='passthrough' ):
    """
    Convert from a cv2 image to a ROS Image message.
    """
    global bridge
    return bridge.cv2_to_imgmsg(cv2_data, image_encoding)

# crops and then resizes to desired shape without distortion
#  based on https://stackoverflow.com/a/4744625/882436
def resize_crop( img, target_w, target_h ):

    target = target_w/float(target_h)
    h,w = img.shape[:2]
    ar = w/float(h)

    if ar  > target: #crop left&right
        new_w = int(target*h)
        offset = ( w - new_w ) / 2
        resize = (offset, 0, w - offset, h)
    else: # crop top&bottom
        new_h = int(w/target)
        offset = (h-new_h) / 2
        resize = (0,offset,w,h-offset)

    result = img[resize[1]:resize[3], resize[0]:resize[2]]  #crop

    return cv2.resize(result, (target_w,target_h)) #resize

# create a float32 message for a 2D matrix
#   based on https://gist.github.com/jarvisschultz/7a886ed2714fac9f5226
""" def convert_numpy_2D_to_ros_float32_msg( np_arr ):
    # publish a 3x3 matrix

    h,w = np_arr.shape[:2]
    
    mat = Float32MultiArray()
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim[0].label = "height"
    mat.layout.dim[1].label = "width"
    mat.layout.dim[0].size = h
    mat.layout.dim[1].size = w
    mat.layout.dim[0].stride = h*w
    mat.layout.dim[1].stride = w
    mat.layout.data_offset = 0
    #mat.data = [0]*(h*w)
    mat.data = np_arr.copy()

    return mat """

# create ros string message from arbitrary dictionary
def convert_dictionary_to_ros_msg( dict ):
    msg = String()
    msg.data = json.dumps(dict)
    return msg

# create dictionary from ros string message
def convert_ros_msg_to_dictionary( msg ):
    if msg._type == 'std_msgs/String':
        return json.loads(msg.data)
    raise TypeError("Unexpected type: " + str(type(msg)))