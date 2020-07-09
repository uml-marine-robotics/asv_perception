#!/usr/bin/env python

import sys, os
import rospy
import numpy as np
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point,Quaternion,TwistWithCovariance,Vector3

# ab3dmot
from AB3DMOT_libs.model import AB3DMOT

class ObstacleTrackingNode(object):
    """
    ROS node which wraps AB3DMOT
        https://github.com/xinshuoweng/AB3DMOT

    Subscribers:
        ~input:  [sensor_msgs/PointCloud2]  Point cloud representing an object, called once per cluster

    Publishers:
        ~output: [ab3dmot_ros/TrackedObject.msg]  TrackedObject for objects which meet the criteria in params

    Parameters:
        ~max_age:   [int, default=2]  AB3DMOT param:  preserve the object if it does not appear no more than N frames, interpolate the detection
        ~min_hits:  [int, default=3]  AB3DMOT param:  minimum number of hits before an object is reported
        ~publish_pointclouds: [bool, default=False]   Flag if most recent object pointcloud will be set within the published TrackedObject (if available/applicable)
        ~linear_velocity_min: [float, default=0.1]    Minimum magnitude of linear velocity that an obstacle must reach before being reported
        ~tf_frame   [string, default=None]            Transform the input into this frame via TF2.  If None, no transform is performed
    """

    def __init__( self ):
        self.node_name = rospy.get_name()
        #self.pub = rospy.Publisher( '~output', TrackedObject, queue_size=1 )
        #self.sub = rospy.Subscriber( '~input', PointCloud2, self.cb_sub, queue_size=10 )
        
        #self.tf_frame = rospy.get_param( '~tf_frame', None )
        #tfBuffer = tf2_ros.Buffer()
        #listener = tf2_ros.TransformListener(tfBuffer)

        self.state = None

        #self.current_stamp = None
        #self.current_stamp_published = False # have we already published data for the current timestamp?
        self.tracked_object_meta = {} # metadata for currently tracked objects.  k=trk id, v=[initial_time]
        
        # ab3dmot data
        self.mot_tracker = AB3DMOT( max_age=rospy.get_param('~max_age', 2), min_hits=rospy.get_param('~min_hits', 3) )
        #self.current_detections = []  # array of [h,w,l,x,y,z,theta] for ab3dmot
        #self.current_detections_meta = [] # meta for current detections

    
if __name__ == "__main__":

    try:
        rospy.init_node(ObstacleTrackingNode.__name__)
        n = ObstacleTrackingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass