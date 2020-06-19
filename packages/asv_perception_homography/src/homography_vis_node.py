#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from calibrate import create_unified_image
from sensor_msgs.msg import Image, CompressedImage, Imu
import asv_perception_utils as utils
from asv_perception_common.msg import Homography
from tf.transformations import euler_from_quaternion

class homography_visualization(object):

    def __init__(self):

        self.node_name = rospy.get_name()
        self.radarImg = None
        self.homography = None
        self.imu = None
        
        # publisher
        self.pub = rospy.Publisher( "~image", Image, queue_size=1)

        # subscribers
        #  currently, not doing any time syncing of inputs.  assuming current information on all inputs
        # todo:  approximatetimesync on radar and image, but not homography

        # radar
        self.sub_radar = rospy.Subscriber( "~radar", Image, self.processRadarImage, queue_size=1, buff_size=2**24 )

        # rgb camera
        self.sub_rgb = rospy.Subscriber( "~rgb", CompressedImage, self.processRGBImage, queue_size=1, buff_size=2**24 )

        # homography matrix
        self.sub_homography = rospy.Subscriber( "~rgb_radar", Homography, self.processHomography, queue_size=1 )

        # imu
        self.sub_imu = rospy.Subscriber( "~imu", Imu, self.processImu, queue_size=1 )

    def processImu( self, msg ):
        self.imu = msg

    def processRadarImage(self, image_msg):
        self.radarImg = utils.convert_ros_msg_to_cv2(image_msg)

    def processHomography( self, msg ):
        # convert float[9] to numpy 3x3
        #   then invert the homography, since we want radar --> image for create_unified_image
        self.homography = np.linalg.inv( np.array(msg.values).reshape((3,3)) )

    def processRGBImage(self, image_msg):

        # no subscribers, no work
        if self.pub.get_num_connections() < 1:
            return

        if self.radarImg is None:
            rospy.loginfo('******** RADAR image not yet received, dropping rgb frame ********')
            return

        if self.homography is None:
            rospy.loginfo('******** Homography not yet received, dropping rgb frame ********')
            return

        image = utils.convert_ros_msg_to_cv2(image_msg)

        result = create_unified_image( image, self.radarImg, self.homography )

        # print some text on the resulting image
        printText = lambda text, position : cv2.putText( result, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 2 )

        # convert imu orientation to euler angles, display
        #  todo:  verify degrees/rads?
        if self.imu is not None:
            q = [self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w]
            rpy = euler_from_quaternion(q)
            printText( "{}: {:.2f}".format( "Roll", rpy[0] ), ( 5, 25 ) )
            printText( "{}: {:.2f}".format( "Pitch", rpy[1] ), ( 5, 50 ) )

        msg = utils.convert_cv2_to_ros_msg( result, 'bgr8' )
        msg.header = image_msg.header # match timestamp
        self.pub.publish( msg )

if __name__ == "__main__":

    try:
        rospy.init_node(homography_visualization.__name__)
        n = homography_visualization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
