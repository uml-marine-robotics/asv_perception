#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Header, Empty
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from asv_perception_common.msg import Homography
from asv_perception_common import utils

from asv_perception_homography.calibrate_utils import create_warp_matrix, get_radar_to_world_matrix

class homography_node(object):

    def __init__(self):
        
        self.node_name = rospy.get_name()
        
        # pitch and roll, degrees   
        self.imu_pitch = 0.
        self.imu_roll = 0.

        self.has_published = False
        
        # publish with latch in case of no IMU/testing/etc
        self.pub_rgb_radarimg = rospy.Publisher( "~rgb_radarimg", Homography, queue_size=1, latch=True)
        self.pub_radarimg_radar = rospy.Publisher( "~radarimg_radar", Homography, queue_size=1, latch=True)
        self.pub_rgb_radar = rospy.Publisher( "~rgb_radar", Homography, queue_size=1, latch=True)

        # subscriptions:

        # imu
        self.sub_imu = rospy.Subscriber( "~imu", Imu, self.cb_imu, queue_size=1 )

        # refresh notification from calibration tool; useful for visualization updates when rosbag is paused/stopped
        self.sub_refresh = rospy.Subscriber( "~refresh", Empty, self.cb_refresh, queue_size=1 )
    
    def cb_refresh( self, msg ):
        self.publish()

    def cb_imu( self, msg ):
        # imu.orientation is a normalized quaternion.  euler_from_quaternion returns radians
        rpy = np.degrees( euler_from_quaternion( [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w] ) )
        self.imu_roll, self.imu_pitch = rpy[:2]
        self.publish()

    # publish from rgb to other frame
    def publish_homography( self, pub, M, t, frame_id, child_frame_id ):
        msg = Homography()
        msg.header.stamp = t
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.values = np.ravel( M )
        pub.publish( msg )
        self.has_published = True


    def publish( self, msg = None ):

        # check for early exit
        if self.has_published and self.pub_rgb_radarimg.get_num_connections() < 1 and self.pub_radarimg_radar.get_num_connections() < 1 and self.pub_rgb_radar.get_num_connections() < 1:
            return

        # message time
        t = rospy.Time.now()

        # rgb to radar
        #  create_warp_matrix computes radar to rgb, we want the inverse
        M_rgb_radar = np.linalg.inv( create_warp_matrix( 
            rospy.get_param('~radar_img_w') 
            , rospy.get_param('~radar_img_w')
            , rospy.get_param('~yaw')
            , rospy.get_param('~pitch') - self.imu_pitch
            , rospy.get_param('~roll') - self.imu_roll
            , 1.
            , rospy.get_param('~fovy')
            , rospy.get_param('~tx')
            , rospy.get_param('~ty')
            , rospy.get_param('~tz')
            ) 
            )

        rgb_frame_id = rospy.get_param("~rgb_frame_id")
        radarimg_frame_id = rospy.get_param("~radarimg_frame_id")
        radar_frame_id = rospy.get_param("~radar_frame_id")

        self.publish_homography( self.pub_rgb_radarimg, M_rgb_radar, t, rgb_frame_id, radarimg_frame_id )

        # radar to robot
        #  multiply radar range by 2 to get diameter
        M_radar_robot = get_radar_to_world_matrix( rospy.get_param('~radar_img_w'), 2. * rospy.get_param('~radar_range') )
        self.publish_homography( self.pub_radarimg_radar, M_radar_robot, t, radarimg_frame_id, radar_frame_id )

        # rgb to robot is (radar_to_robot)*(rgb_to_radar)
        M_rgb_robot = np.matmul( M_radar_robot, M_rgb_radar )
        self.publish_homography( self.pub_rgb_radar, M_rgb_robot, t, rgb_frame_id, radar_frame_id )

if __name__ == "__main__":

    try:
        rospy.init_node(homography_node.__name__)
        n = homography_node()
        n.publish()  # publish one time with latch, subsequent publishes will be triggered by receipt of IMU data
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
