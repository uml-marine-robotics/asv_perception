#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import math, sys, os, copy
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point,Quaternion,TwistWithCovariance,Vector3,PoseStamped,PoseWithCovariance, Pose
from asv_perception_common.msg import ObstacleArray, Obstacle
from asv_perception_common.NodeLazy import NodeLazy
from asv_perception_common.FrameTransformer import FrameTransformer

# smstf
from smstf.tracking import SensorTracking, TrackedObject, EuclideanDistance, HellingerDistance, BhattacharyyaDistance, PerspectiveDistance

def create_tracked_object( obstacle, min_dim=1., **kwargs ):
    """ 
    Creates smstf.TrackedObject 
    Generate measurements [x,y,z,l,w,h]
    l=x,w=y,h=z
    ensures each of (l,w,h) >= min_dim => volume > 0
    """
    measurements = [
        obstacle.pose.pose.position.x
        , obstacle.pose.pose.position.y
        , obstacle.pose.pose.position.z
        , max( min_dim, obstacle.dimensions.x )
        , max( min_dim, obstacle.dimensions.y )
        , max( min_dim, obstacle.dimensions.z )
    ]
    result = TrackedObject( measurements, obstacle, id=obstacle.id, **kwargs )

    # if input obstacle has covariance data, use it; otherwise leave defaults
    if obstacle.pose.covariance[0] > 0.:
        covar = np.array(obstacle.pose.covariance,dtype=float).reshape((6,6))
        result.filter.P[:3,:3]=covar[:3,:3]

    if obstacle.velocity.covariance[0] > 0.:
        covar = np.array(obstacle.velocity.covariance,dtype=float).reshape((6,6))
        result.filter.P[6:9,6:9]=covar[:3,:3]

    result.filter.x[6]=obstacle.velocity.twist.linear.x
    result.filter.x[7]=obstacle.velocity.twist.linear.y
    result.filter.x[8]=obstacle.velocity.twist.linear.z

    return result
    

def update_obstacle_data( trk ):
    """ 
    Updates the obstacle estimate stored in the SMSTF.TrackedObject 
        using the current tracker state, latest obstacle estimate, and tracker filter data
    Merge all info into the Obstacle at trk.data[0], remove latest obstacle data when done
    """
    assert(trk.data and len(trk.data))

    # latest observation could be in the past if obstacle is being extrapolated.  

    result = trk.data[0]
    result.id = trk.id

    # merge best data from last observation, if have multiple observations
    if len(trk.data) > 1:
        latest = trk.data[-1]

        # label, label_probability
        if latest.label_probability > result.label_probability:
            result.label = latest.label
            result.label_probability = latest.label_probability

        # hull2d, area.  use latest info
        result.hull2d = latest.hull2d
        result.area = latest.area

    if result.observed_initial.is_zero():  # haven't seen before, set initial observation time
        result.observed_initial = result.header.stamp

    # update latest observation time, assume observed at current time, even if extrapolated
    result.header.stamp = rospy.Time.now()

    # compute object lifetime in float seconds
    dt = ( result.header.stamp - result.observed_initial ).to_sec()

    # get state data from tracker.  [x,y,z,l,w,h,xdot,ydot,zdot]
    s = trk.filter.x

    # posewithcovariance
    result.pose.pose.position=Point( float(s[0]), float(s[1]), float(s[2]) )
    pose_covar = np.zeros((6,6))
    pose_covar[:3,:3] = trk.filter.P[:3,:3]
    result.pose.covariance=np.ravel(pose_covar)

    # estimated dimensions
    result.dimensions = Vector3( float(s[3]), float(s[4]), float(s[5]) )

    # linear velocity and orientation
    #   convert linear velocity to m/s; currently measured in whatever frequency we get a frame change in cb_sub
    #   don't report for initial observation
    if dt > 0:
        lv = np.copy( s[6:9] )
        lv *= float(trk.hits + trk.age) / dt       # hits+age = frame count.  convert to m/s
        result.velocity.twist.linear = Vector3( float(lv[0]), float(lv[1]), float(lv[2]) )
        twist_covar = np.zeros((6,6))
        twist_covar[:3,:3] = trk.filter.P[6:9,6:9]
        result.velocity.covariance = np.ravel( twist_covar )

        # orientation:  assume in direction of linear velocity
        v = result.velocity.twist.linear
        pitch = np.arctan2( v.z, abs( v.x ) )
        yaw = np.arctan2( v.y, v.x )

        result.pose.pose.orientation = Quaternion( *quaternion_from_euler( 0, pitch, yaw ) )
        # TODO: rotate hull2d points to account for orientation; will affect downstream clients
    
    # only keep merged obstacle data
    trk.data=[result]


class ObstacleTrackingNode( NodeLazy ):
    """
    ROS node which wraps SMSTF

    Subscribers:
        ~input:     [asv_perception_common/ObstacleArray.msg]  
                    Candidate obstacles for tracking.  Obstacle pose.position and dimensions should be populated

    Publishers:
        ~obstacles: [asv_perception_common/ObstacleArray.msg]  
                    Tracked obstacles.  The following obstacle fields will be computed/populated:
                        - id
                        - observed_initial
                        - pose.orientation (derived from velocity)
                        - dimensions
                        - velocity.linear
                    if ~tf_frame is specified, obstacle will be transformed to specified frame

    Parameters:
        ~max_age:   [int, default=2]  preserve the object if it does not appear in <= N frames, interpolate the detection
        ~min_hits:  [int, default=3]  minimum number of hits before an object is reported
        ~tf_frame:          [string, default=None]  Transform the input into this frame via TF2.  If None, no transform is performed
        ~tf_time_current:   [bool, default=False]  Perform transforms using current time rather than original time
        ~cost_fn:       [string, default='euclidean']
                        cost function for evaluating obstacle association
                        supported values:
                            - euclidean:     euclidean distance between centroids
                            - hellinger:     hellinger distance using means and covariances
                            - bhattacharyya: bhattacharyya distance using means and covariances
        ~cost_fn_min:   [float, default=0.]     minimum match value for the cost function
        ~cost_fn_max:   [float, default=10.]    maximum match value for the cost function
        ~measurement_uncertainty    [float, default=None]  Kalman filter measurement uncertainty
    """

    def __init__( self ):
        
        self.node_name = rospy.get_name()
        self.sub = None

        # tracking
        self.tracker = None
        self.min_hits = rospy.get_param( '~min_hits', 3 )
        self.measurement_uncertainty = rospy.get_param('~measurement_uncertainty', None )
        self.estimate_velocity=rospy.get_param( '~estimate_velocity', True )

        # tf stuff
        self.ft = None  # FrameTransformer
        self.tf_frame = rospy.get_param('~tf_frame', None )

        # init tracker
        # select cost function
        cost_fn = { 
            'euclidean': EuclideanDistance(), 
            'hellinger': HellingerDistance(), 
            'bhattacharyya': BhattacharyyaDistance(),
            'perspective': PerspectiveDistance()
            }.get( rospy.get_param('~cost_fn', 'euclidean') )

        if cost_fn is None:
            raise NotImplementedError(rospy.get_param('~cost_fn'))

        self.tracker = SensorTracking( 
            max_age=rospy.get_param( '~max_age', 2 ),  
            cost_fn=cost_fn, 
            cost_fn_min=rospy.get_param('~cost_fn_min', 0. ),
            cost_fn_max=rospy.get_param('~cost_fn_max', 10. )
            )

        # obstacle publisher
        self.pub = self.advertise( '~obstacles', ObstacleArray, queue_size=1 )
        
        # init subs
        self.sub = rospy.Subscriber( '~input', ObstacleArray, callback=self.cb_sub, queue_size=1, buff_size=2**24 )

        print("tracking node: {0} created".format(self.node_name))
        
    def subscribe( self ):

        print("tracking node {0} subscriber invoked (start).".format(self.node_name))
        # init tf
        if not self.tf_frame is None and len(self.tf_frame) > 0:
            self.ft = FrameTransformer()
            self.ft.use_most_recent_tf = rospy.get_param('tf_time_current', False )

        print("tracking node {0} subscriber invoked (end).".format(self.node_name))

    def unsubscribe( self ):

        if self.sub:
            self.sub.unregister()
            self.sub = None

        self.ft = None
        self.tracker=None

    def transform_msg( self, msg ):
        """ if needed, performs tf transforms on input message """
        
        # will not have frame transformer if no tf frame specified
        if self.ft is None:
            print("No frame transformer...")
            return

        msg.header.frame_id = self.tf_frame

        for obs in msg.obstacles:
            self.ft.transform_obstacle( obs, self.tf_frame )

    def cb_sub( self, msg ):        

        print("tracking node {0} cb_sub invoked (start).".format(self.node_name))
        print("Number of obstacles received={0}, nodename={1}".format(len(msg.obstacles), self.node_name))

        for oneObs in msg.obstacles:
            print("oneObs.nodename={0}, oneObs.position={1}, {2}, {3}, oneObs.Size={4}, {5}, {6}".format(self.node_name, oneObs.pose.pose.position.x, oneObs.pose.pose.position.y,
                                                  oneObs.pose.pose.position.z, oneObs.dimensions.x,
                                                  oneObs.dimensions.y, oneObs.dimensions.z))
        # perform tf transforms on input msg if needed
        self.transform_msg( msg )

        # if using PerspectiveDistance cost fn, set the latest origin
        if self.ft and isinstance( self.tracker.cost_fn, PerspectiveDistance ):
            pos = self.ft.transform_pose( Pose(), dst_frame=self.tf_frame )
            self.tracker.cost_fn.origin = np.array( [pos.position.x,pos.position.y,pos.position.z ])

        # create list of smstf.TrackedObject
        trackers = self.tracker.update( 
            [ create_tracked_object( 
                obs, measurement_uncertainty=self.measurement_uncertainty, estimate_velocity=self.estimate_velocity 
                ) 
                for obs in msg.obstacles 
            ] 
            )

        result = ObstacleArray()
        result.header = msg.header

        print("Num of trackers created = {0}".format(len(trackers)))
        for trk in trackers:
            update_obstacle_data( trk ) # perform obstacle data update

            if trk.hits >= self.min_hits:
                result.obstacles.append( trk.data[0] )

        
        print("Print just before publishing of result")
        self.pub.publish( result )
        print("After update_obstacle_data, result.obstacles={0}, node={1}".format(len(result.obstacles), self.node_name))
        print("tracking node {0} cb_sub invoked (end).".format(self.node_name))
    
if __name__ == "__main__":

    try:
        rospy.init_node(ObstacleTrackingNode.__name__)
        n = ObstacleTrackingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass