#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import rospy
import numpy as np
import math
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu, Image, PointCloud2, CompressedImage
from asv_perception_common.msg import Homography
from asv_perception_common import utils
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array

import cv2
from asv_perception_common.NodeLazy import NodeLazy
from asv_perception_homography.homography_utils import (
    Exemplar, standardize_points, to_gpu, transform_points,
    optimize, create_unified_image, get_adjusted_params, 
    create_camera_to_metric_matrix, create_metric_to_camera_matrix,
    estimate_pixel_to_metric_size
)
from asv_perception_homography.homographyagent import HomographyAgent

from asv_perception_common.msg import Classification, ClassificationArray
from asv_perception_common import utils

class SimulatedService( object ):
    "Simulate a ROS service call.  Hack"

    def __init__( self ):
        self.response = None

    def cb( self, msg ):
        self.response = msg

    def __call__( self, topic, data_class, **kwargs ):
        
        rate = rospy.Rate(50)

        sub = rospy.Subscriber( topic, data_class, callback=self.cb, **kwargs )
        
        while not self.response:
            rate.sleep()

        sub.unregister()

        return self.response


def print_image_text( img, u, v, d, color = (255,255,255), bgcolor=(255,0,0) ):
    
    #TEXT_HEIGHT = 25
    #TEXT_WIDTH = 400

    if not d:
        return

    i = 0
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.75
    font_thickness = 2
    # todo:      text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
                #text_w, text_h = text_size

    text_h = 25
    text_w = 500
    print_text = lambda text : cv2.putText( img, text, ( u, v + -i*text_h ), font, font_scale, color, font_thickness )

    # background
    cv2.rectangle(img, ( u, max( v - text_h*len(d), 0 ) ), ( u+text_w, v ), (255,0,0), -1)

    for el in d:
        print_text( el )
        i += 1

def annotate( img, clsMsg, M ):
    """
    Append classification/localization annotations to image
    """

    # do color conversion rgb --> bgr (optional)
    #img = cv2.cvtColor( img, cv2.COLOR_RGB2BGR )

    for cls in clsMsg.classifications:
        (w, h) = (cls.roi.width, cls.roi.height)
        (x, y) = ( cls.roi.x_offset, cls.roi.y_offset )
        
        # draw a bounding box rectangle and label on the image
        color = (255,0,0)  # B G R
        cv2.rectangle( img, (x, y), (x + w, y + h), color, 2)
        text = "{} ({:.2f})".format( cls.label, cls.probability )
        #cv2.putText( img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2 )

        # convert image coordinates to metric; use bottom 2 points of bounding box
        pts = np.array([[x,y+h,1],[x+w,y+h,1]],dtype=np.float32)  # x, y, 1
        pts_metric = transform_points( pts, M )
        mean_x, mean_y = np.mean(pts_metric,axis=0)[:2]
        location_text = "x={:.1f}, y={:.1f}".format( mean_x, mean_y, math.sqrt( mean_x**2 + mean_y**2 ) )
        
        # calculate height 
        px_sz = estimate_pixel_to_metric_size( (2*x+w)//2, y+h, M )
        info_text = "d={:.1f}, h={:.1f}".format( math.sqrt( mean_x**2 + mean_y**2 ), h*px_sz )

        print_image_text( img, x, y-5, [ location_text, info_text ] )


class homography_node( NodeLazy ):
    """
    Node to autocalibrate camera and radar.  
    Runs after receives a full pointcloud refresh.  
    TODO:  update radar/pointcloud publisher to have customizable offset (eg 90 degrees) so we can ensure we get the latest front view
    Parameters:
        - ~use_gpu:      [bool]      flag to use GPU for processing
        - ~max_frames:   [int]       number of frames to use for calibration
        - ~max_error:    [float]     maximum frame error to allow before recalibration
        - ~rate:         [float]     Hz at which the current error is evaluated
        - ~src_frame:    [string]    homography msg source frame
        - ~dst_frame:    [string]    homography msg destination frame
        - ~calibration_[lb/ub]  [string]  comma-separated string containing values for parameter optimization limits, in the order:
                                        roll pitch yaw fov tx ty tz imu_roll imu_roll_velocity imu_pitch imu_pitch_cubed imu_pitch_velocity
        - ~points_range:    [int]       maximum range of the input pointcloud

    Subscriptions:
        - ~imu:         [sensor_msgs/Imu]   IMU data, ENU
        - ~points:      [sensor_msgs/PointCloud2]   Pointcloud points, in robot frame
        - ~rgb:         [sensor_msgs/Image]   RGB camera image for visualization
        - ~obstacles:   [sensor_msgs/Image]  Segmentation image containing only obstacle pixels
        - ~classifications:  [asv_perception_common/ClassificationArray]  Array of detections

    Publications:
        - ~camera_metric    [asv_perception_common/Homography]  Homography matrix from camera frame to metric frame (2D)
        - ~debug            [sensor_msgs/Image]  Debug image for optimization
        - ~vis              [sensor_msgs/Image]   Visualization of points over RGB image
    """

    def __init__(self):

        self.last_imu_msg = None
        self.last_pts_msg = None
        self.last_cls_msg = None
        self.last_camera_to_metric = None  # last pub camera_to_metric matrix

        self.use_gpu = bool( rospy.get_param("~use_gpu", True ) )
        self.max_frames = rospy.get_param("~max_frames", 5 )
        self.max_error = rospy.get_param("~max_error", 1000. )
        
        self.src_frame = rospy.get_param("~src_frame", "camera" )
        self.dst_frame = rospy.get_param("~dst_frame", "base_link" )

        # node active state
        self.is_active = False

        # params:
                
        # get float[] from comma-separated list of floats
        parse_floats = lambda s : np.array([ float( v.strip() ) for v in str(s).split(',') ], dtype=np.float32 )

        self.calibration_lb = parse_floats( rospy.get_param("~calibration_lb" ) )
        self.calibration_ub = parse_floats( rospy.get_param("~calibration_ub" ) )
        self.points_range = rospy.get_param("~points_range")

        # homography agent
        self.agent = HomographyAgent( self.calibration_lb, self.calibration_ub, self.points_range, max_frames=self.max_frames, max_error=self.max_error )

        # subscriptions:
        self.subs = []

        # publications
        self.pub_camera_metric = self.advertise( "~camera_metric", Homography, queue_size=1 )
        self.pub_debug = self.advertise( "~debug", Image, queue_size=1 )  # debug optimization visualization
        self.pub_vis = self.advertise( "~vis", Image, queue_size=1 )  # points/rgb visualization

    def subscribe( self ):
        
        self.unsubscribe()

        rospy.logwarn("Subscribing")

        # subscribers; approximate time sync seems to fail when restarting a rosbag; just use latest of each msg

        # points
        self.subs.append( rospy.Subscriber( "~points", PointCloud2, self.cb_points, queue_size=1 ) )

        # rgb
        self.subs.append( rospy.Subscriber( "~rgb", CompressedImage, self.cb_rgb, queue_size=1, buff_size=2**24 ) )

        # imu
        self.subs.append( rospy.Subscriber( "~imu", Imu, self.cb_imu, queue_size=1 ) )

        # classifications
        self.subs.append( rospy.Subscriber( "~classifications", ClassificationArray, self.cb_cls, queue_size=1 ) )

        # hack: separate thread for training
        self.subs.append( rospy.Subscriber( "~train", Empty, self.cb_train, queue_size=1))

        self.is_active=True

    def unsubscribe( self ):

        self.is_active=False 
        for sub in self.subs:
            sub.unregister()
        self.subs = []

    def cb_imu( self, msg ):
        self.last_imu_msg = msg

        # if we have params, publish a new matrix for camera to metric conversion
        if ( self.pub_camera_metric.get_num_connections() > 0 or self.pub_vis.get_num_connections() > 0 ) and not self.agent is None and len(self.agent.best_params) > 0:
            self.last_camera_to_metric = self.agent.create_camera_to_metric_matrix( self.last_imu_msg )
            self.publish_homography( self.pub_camera_metric, self.last_camera_to_metric, rospy.Time.now(), self.src_frame, self.dst_frame )

    def cb_points( self, msg ):
        self.last_pts_msg = msg

    def cb_cls( self, msg ):
        self.last_cls_msg = msg

    def cb_rgb( self, msg ):
        
        if self.pub_vis.get_num_connections() < 1 or self.agent is None or len( self.agent.best_params ) == 0:
            return

        rgb = utils.convert_ros_msg_to_cv2( msg )
        metric_camera = self.agent.create_metric_to_camera_matrix( self.last_imu_msg )
        pts_np = standardize_points( pointcloud2_to_xyz_array( self.last_pts_msg ) )
        
        if self.use_gpu:
            pts_np = to_gpu(pts_np)
            metric_camera = to_gpu(metric_camera)

        rgb = create_unified_image( rgb, pts_np, metric_camera )

        if self.last_cls_msg and not ( self.last_camera_to_metric is None ):
            annotate( rgb, self.last_cls_msg, self.last_camera_to_metric )

        result = utils.convert_cv2_to_ros_msg( rgb, 'bgr8' )
        result.header = msg.header

        self.pub_vis.publish( result )


    # publish Homography message
    def publish_homography( self, pub, M, t, frame_id, child_frame_id ):
        msg = Homography()
        msg.header.stamp = t
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.values = np.ravel( M )
        pub.publish( msg )

    def cb_train( self, _ ):
        "Perform training if needed"

        if not self.is_active or self.agent is None:
            return

        # HomographyAgent handles train decision, frame queue locking
        self.agent.train()

    def __call__( self ):
        "Handle error evaluation and parameter selection"

        # waiting for data, not active
        if not self.is_active or self.agent is None or self.last_imu_msg is None or self.last_pts_msg is None:
            return

        # collect the current data
        seg = utils.convert_ros_msg_to_cv2( SimulatedService()( "~obstacles", Image, queue_size=1, buff_size=2**24 ) )
        pts_np = pointcloud2_to_xyz_array( self.last_pts_msg )

        # create the exemplar
        current_frame = Exemplar( pts_np, seg, self.last_imu_msg, self.points_range, self.calibration_lb, self.calibration_ub, pts_min_x = 4., use_gpu = self.use_gpu )
        
        # perform eval; this will update agent best_params and set XXX_needs_training flags
        self.agent.evaluate( current_frame )

    def publish_debug_visualization( self, seg_img, pts, pts_to_img ):
        "Publish debug visualization of transformed points on seg image"

        if self.pub_debug.get_num_connections() < 1:
            return

        dbg_img = create_unified_image( seg_img, pts, pts_to_img )

        self.pub_debug.publish( utils.convert_cv2_to_ros_msg( dbg_img, 'bgr8' ) )

    
if __name__ == "__main__":

    try:
        rospy.init_node(homography_node.__name__)
        n = homography_node()
        
        rate = rospy.Rate( rospy.get_param("~rate", 1.0 ) )

        # HACK separate training thread
        train_pub = rospy.Publisher("~train", Empty, queue_size=1 )

        while True:
            n()
            train_pub.publish()
            rate.sleep()
        #rospy.spin()

    except rospy.ROSInterruptException:
        pass
