#!/usr/bin/env python

import sys, os, copy
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point,Quaternion,TwistWithCovariance,Vector3,PoseStamped
from asv_perception_common.msg import ObstacleArray, Obstacle
from asv_perception_common.NodeLazy import NodeLazy
from asv_perception_common.FrameTransformer import FrameTransformer

# ab3dmot
from AB3DMOT_libs.model import AB3DMOT

def create_ab3dmot_detection( obstacle, min_dim=1. ):
    """ 
    Convert obstacle to [h,w,l,x,y,z,theta] (in camera frame) for ab3dmot 
    ensures each of (h,w,l) >= min_dim (so volume > 0)
    obstacle frame to camera frame:  y --> x/l;  x --> z/w;  z --> y/h
    """
    return [
        max( min_dim, obstacle.dimensions.z )
        , max( min_dim, obstacle.dimensions.x )
        , max( min_dim, obstacle.dimensions.y )
        , obstacle.pose.pose.position.y
        , obstacle.pose.pose.position.z
        , obstacle.pose.pose.position.x
        , 0  # not using input orientation
    ]

class TrackerState( object ):
    """ represents tracker state """

    def __init__( self, max_age, min_hits, tf_frame, tf_time_current ):
        
        self.ft = None  # FrameTransformer

        if not tf_frame is None and len(tf_frame) > 0:
            self.ft = FrameTransformer()
            self.ft.use_most_recent_tf = tf_time_current
        
        self.mot_tracker = AB3DMOT( max_age=max_age, min_hits=min_hits )
        self.tracked_obstacles = {}


class ObstacleTrackingNode( NodeLazy ):
    """
    ROS node which wraps AB3DMOT
        https://github.com/xinshuoweng/AB3DMOT

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
                    if ~tf_frame is specified, obstacle pose and points will be transformed to specified frame

    Parameters:
        ~max_age:   [int, default=2]  AB3DMOT param:  preserve the object if it does not appear in <= N frames, interpolate the detection
        ~min_hits:  [int, default=3]  AB3DMOT param:  minimum number of hits before an object is reported
        ~tf_frame:          [string, default=None]  Transform the input into this frame via TF2.  If None, no transform is performed
        ~tf_time_current:   [bool, default=False]  Perform transforms using current time rather than original time
    """

    def __init__( self ):
        
        self.node_name = rospy.get_name()
        self.sub = None

        # tf stuff
        self.tf_frame = rospy.get_param('~tf_frame', None )
        self.tf_time_current = rospy.get_param('tf_time_current', False )
        self.max_age = rospy.get_param( '~max_age', 2 )
        self.min_hits = rospy.get_param( '~min_hits', 3 )

        self.state = None

        self.pub = self.advertise( '~obstacles', ObstacleArray, queue_size=1 )

    def subscribe( self ):

        self.state = TrackerState( self.max_age, self.min_hits, self.tf_frame, self.tf_time_current )
        self.sub = rospy.Subscriber( '~input', ObstacleArray, callback=self.cb_sub, queue_size=1, buff_size=2**24 )

    def unsubscribe( self ):

        if not self.sub is None:
            self.sub.unregister()
            self.sub = None

        self.state = None

    def transform_msg( self, msg ):
        """ if needed, performs tf transforms on input message """
        
        # tf frame not specified, or already in correct frame
        if self.tf_frame is None or len(self.tf_frame) == 0:
            return

        msg.header.frame_id = self.tf_frame

        for obs in msg.obstacles:
            self.state.ft.transform_obstacle( obs, self.tf_frame )

    def create_obstacle_estimate( self, tracker, prev ):
        """ create a new obstacle estimate using the current obstacle data, the estimate from ab3dmot, and previous estimate """

        # tracker.info[0]:  most recent observation, could be in the past if obstacle is being extrapolated
        result = tracker.info[0]
        result.id = str(tracker.id)
        
        # merge best data from prev, current
        if not prev is None:
            result.observed_initial = prev.observed_initial    
            if prev.label_probability > result.label_probability:
                result.label = prev.label
                result.label_probability = prev.label_probability

        if result.observed_initial.is_zero():  # haven't seen before, set initial observation time
            result.observed_initial = result.header.stamp

        # update latest observation time, assume observed at current time, even if extrapolated
        result.header.stamp = rospy.Time.now()

        # compute object lifetime in float seconds
        dt = ( result.header.stamp - result.observed_initial ).to_sec()

        # get state data from tracker
        s = tracker.kf.x

        # camera frame to obstacle frame:  x/l --> y;  z/w --> x;  y/h --> z
        # order in s:  [x,y,z,theta,l,w,h,xdot,ydot,zdot]

        # posewithcovariance
        result.pose.pose.position=Point( float(s[2]), float(s[0]), float(s[1]) )
        pose_covar = np.zeros((6,6))
        # reorder covariance vals; only generated on the diagonal
        pp = tracker.kf.P[:3,:3]
        pose_covar[0][0] = pp[2][2]
        pose_covar[1][1] = pp[0][0]
        pose_covar[2][2] = pp[1][1]
        result.pose.covariance = np.ravel( pose_covar )

        # estimated dimensions
        result.dimensions = Vector3( float(s[5]), float(s[4]), float(s[6]) )

        # linear velocity and orientation
        #   convert linear velocity to m/s; currently measured in whatever frequency we get a frame change in cb_sub
        #   don't report for initial observation
        if dt > 0:
            lv = np.copy( s[7:10] )
            lv *= float(tracker.age) / dt       # convert to m/s
            result.velocity.twist.linear = Vector3( float(lv[2]), float(lv[0]), float(lv[1]) )  # reorder
            twist_covar = np.zeros((6,6))
            #twist_covar[:3,:3] = tracker.kf.P[7:10,7:10]
            # reorder covariance vals; only generated on the diagonal
            vp = tracker.kf.P[7:10,7:10]
            twist_covar[0][0] = vp[2][2]
            twist_covar[1][1] = vp[0][0]
            twist_covar[2][2] = vp[1][1]
            result.velocity.covariance = np.ravel( twist_covar )

            # orientation:  assume in direction of linear velocity
            v = result.velocity.twist.linear
            pitch = np.arctan2( v.z, abs( v.x ) )
            yaw = np.arctan2( v.y, v.x )

            result.pose.pose.orientation = Quaternion( *quaternion_from_euler( 0, pitch, yaw ) )

        return result
            

    def cb_sub( self, msg ):

        # perform tf transforms on input msg if needed
        self.transform_msg( msg )

        dets = np.zeros( (0,7) )
        dets_info = np.zeros( (0,1) )

        if len( msg.obstacles ) > 0:
            dets = [ create_ab3dmot_detection( obs ) for obs in msg.obstacles ]
            dets_info = [ [obs] for obs in msg.obstacles ]

        self.state.mot_tracker.update( {
            'dets': np.asarray( dets )
            , 'info': np.asarray( dets_info, dtype=object )
            } )

        # iterate over updated ab3dmot trackers.  old trackers are removed in 'update' fn
        #   maintain tracked obstacles for active tracks, remove the rest
        #   only publish obstacles which hits exceed min
        pub_msg = ObstacleArray()
        current_obstacles = {}

        for trk in self.state.mot_tracker.trackers:

            # maintain/create tracked obstacle data in current set
            # combine the current obstacle data in trk.info, the estimate from ab3dmot, and any existing data we may have had previously
            obs = self.create_obstacle_estimate( trk, self.state.tracked_obstacles.get( trk.id ) )

            current_obstacles[ trk.id ] = obs
            if trk.hits >= self.state.mot_tracker.min_hits:
                pub_msg.obstacles.append( obs )

        self.state.tracked_obstacles = current_obstacles # update tracked obstacles dict

        # publish tracked obstacles msg
        pub_msg.header = msg.header
        if not self.tf_frame is None and len(self.tf_frame) > 0:
            pub_msg.header.frame_id = self.tf_frame
        self.pub.publish( pub_msg )
    
if __name__ == "__main__":

    try:
        rospy.init_node(ObstacleTrackingNode.__name__)
        n = ObstacleTrackingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass