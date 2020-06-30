#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, Imu
from tf.transformations import euler_from_quaternion

from asv_perception_common.msg import Homography
import asv_perception_utils as utils
from calibrate_utils import create_unified_image

class homography_visualization(object):

    def __init__(self):

        self.node_name = rospy.get_name()
        self.radar_img = None
        self.rgb_img = None
        self.homography = None
        self.imu = None
        self.pub_header = None

        # publisher
        self.pub = rospy.Publisher( "~image", Image, queue_size=1)

        # subscribers; approximate time sync seems to fail when restarting a rosbag; just use latest of each msg
        self.sub_rgb = rospy.Subscriber( "~rgb", CompressedImage, self.cb_rgb, queue_size=1, buff_size=2**24 )
        self.sub_radar = rospy.Subscriber( "~radarimg", Image, self.cb_radar, queue_size=1, buff_size=2**24 )

        # homography matrix
        self.sub_homography = rospy.Subscriber( "~rgb_radarimg", Homography, self.cb_homography, queue_size=1 )

        # imu
        self.sub_imu = rospy.Subscriber( "~imu", Imu, self.cb_imu, queue_size=1 )

    def cb_imu( self, msg ):
        self.imu = msg
        self.publish()

    def cb_rgb( self, msg ):
        self.rgb_img = utils.convert_ros_msg_to_cv2( msg )
        self.pub_header = msg.header
        self.publish()
    
    def cb_radar(self, msg ):
        self.radar_img = utils.convert_ros_msg_to_cv2( msg )
        self.publish()

    def cb_homography( self, msg ):
        # convert float[9] to numpy 3x3
        #   then invert the homography, since we want radar --> rgb for image creation
        self.homography = np.linalg.inv( np.array(msg.values).reshape((3,3)) )
        self.publish()

    def publish(self):

        # no subscribers/data, no work
        if self.pub.get_num_connections() < 1 or self.radar_img is None or self.rgb_img is None or self.homography is None:
            return

        # using rgb msg header
        assert self.pub_header is not None

        img = create_unified_image( self.rgb_img, self.radar_img, self.homography )

        # print some text on the resulting image
        print_text = lambda text, position : cv2.putText( img, text, position, cv2.FONT_HERSHEY_SIMPLEX, 1., (0,255,0), 2 )

        # convert imu orientation to euler angles, display
        if self.imu is not None:
            # imu.orientation is a normalized quaternion.  euler_from_quaternion returns radians
            rpy = euler_from_quaternion( [self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w] )
            rpy = np.degrees( rpy )
            print_text( "{}: {:.2f}".format( "Roll (deg)", rpy[0] ), ( 5, 50 ) )
            print_text( "{}: {:.2f}".format( "Pitch (deg)", rpy[1] ), ( 5, 100 ) )
            print_text( "{}: {:.2f}".format( "Yaw (deg)", rpy[2] ), ( 5, 150 ) )

        msg = utils.convert_cv2_to_ros_msg( img, 'bgr8' )
        msg.header = self.pub_header # match timestamp
        self.pub.publish( msg )

if __name__ == "__main__":

    try:
        rospy.init_node(homography_visualization.__name__)
        n = homography_visualization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
