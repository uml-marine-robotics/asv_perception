#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import copy, sys
from copy import deepcopy
import numpy as np
import rospy
from geometry_msgs.msg import Point,Quaternion,TwistWithCovariance,Vector3,PoseStamped,Pose,Polygon,Point32
from asv_perception_common.msg import ObstacleArray
from asv_perception_common.NodeLazy import NodeLazy
from asv_perception_common.FrameTransformer import FrameTransformer
from asv_perception_tracking.smstf.tracking import EuclideanDistance, HellingerDistance, BhattacharyyaDistance, PerspectiveDistance
from asv_perception_tracking.smstf.fusion import SensorFusion
from tracking import create_tracked_object

def create_obstacle( group_tracker ):
    """ create an obstacle msg from a smstf.TrackedObjectGroup """

    #   TODO:  fusion logic for area, dimensions, hull2d, velocity
    #       header:  latest
    #       observed_initial:  earliest
    #       label, label_probability:  highest, preserve if exists
    #       area, dimensions, hull2d:  from sensor with largest area
    #       velocity:   from sensor with lowest velocity covar
    #       - input covariances are (always?) diagonal matrices with the same value down the diagonal
    #       - TODO:  move velocity calc to smstf
    #   use group.data to store last generated obstacle, useful for "sticky" attributes

    if len(group_tracker.trackers) < 1:
        return None

    # construct a list of Obstacles from all trackers in group
    #   Obstacle message stored in data list
    obstacles = [ trk[1].data[0] for trk in group_tracker.trackers ]

    assert(len(obstacles)>0)

    result = copy.deepcopy(obstacles[0])
    result.id = group_tracker.id  # always use group id
    prev = group_tracker.data # previously generated obstacle, may be None

    # use fused position, position_covar from group
    result.pose.pose.position=Point( float(group_tracker.position[0]), float(group_tracker.position[1]), float(group_tracker.position[2]) )
    pose_covar = np.zeros((6,6))
    pose_covar[:3,:3] = group_tracker.position_covariance
    result.pose.covariance=np.ravel(pose_covar)

    hull2d_source_position = None

    # if we have multiple obstacles, merge to result
    for _, obs in enumerate(obstacles, 1):

        # header
        if obs.header.stamp > result.header.stamp:
            result.header = obs.header

        # observed_initial:  use earliest
        result.observed_initial = min( result.observed_initial, obs.observed_initial )
        
        # label, label_prob
        if obs.label_probability > result.label_probability:
            result.label_probability = obs.label_probability
            result.label = obs.label

        # area, dimensions, hull2d
        if obs.area > result.area:
            result.area = obs.area
            result.dimensions = obs.dimensions
            result.hull2d = obs.hull2d
            hull2d_source_position = obs.pose.pose.position

        # velocity-based data
        if obs.velocity.covariance[0] < result.velocity.covariance[0]:
            result.velocity = obs.velocity
    
    # restore any "sticky" attributes from previously-generated obstacle
    if prev:
        # classification:  use highest observed
        if prev.label_probability > result.label_probability:
            result.label_probability=prev.label_probability
            result.label=prev.label

    # move the points in the 2d convex hull by the distance between the source obstacle and the fused position
    if hull2d_source_position:

        p0 = hull2d_source_position
        p1 = result.pose.pose.position
        delta = ( p0.x-p1.x, p0.y-p1.y, p0.z-p1.z )
        ch = Polygon()
        for pt in result.hull2d.points:
            ch.points.append( Point32( pt.x + delta[0], pt.y + delta[1], pt.z + delta[2] ) )
        result.hull2d=ch

    """
    # for evaluation:  append obstacle_sensor_ids to classification label so it can be see in rviz
    # rebuild the label so we don't keep old sensor info, but we preserve the important label data
    
    # list of sensorid, obstacle_id (from sensor tracker)
    obstacle_sensor_ids = [ ( trk[0], trk[1].data[0].id ) for trk in group_tracker.trackers ]

    if result.label:
        result.label = str(result.label).split( " [", 1 )[0]

    for osi in obstacle_sensor_ids:
        s = " [{}={}]".format( osi[0], osi[1] )
        if s not in result.label:
            result.label += s
    """

    group_tracker.data = result # save generated obstacle to group tracker
    return result

class ObstacleFusionNode( NodeLazy ):
    """ 
        Node for tracking and fusion of obstacle data

        Subscriptions:
            ~[0...n]/input:  [asv_perception_common/ObstacleArray.msg] input ObstacleArray

        Publications:
            ~obstacles:     [asv_perception_common/ObstacleArray.msg] fused ObstacleArray

        Parameters:
            ~n_subs:        [int, default=2]        nbr of input subscribers
            ~publish_rate:  [float, default=5.0]    rate, in Hz, of publishing frequency

            ~sensor0,~sensor1,...~sensor[n_subs-1]:
                dict of:
                    cost_fn:       [string, default='euclidean']
                                    cost function for evaluating obstacle association
                                    supported values:
                                        - euclidean:     euclidean distance between centroids
                                        - hellinger:     hellinger distance using means and covariances
                                        - bhattacharyya: bhattacharyya distance using means and covariances
                                        - perspective:   perspective weighted proportional distance
                    cost_fn_min:   [float, default=0.]     minimum match value for the cost function
                    cost_fn_max:   [float, default=10.]    maximum match value for the cost function
    """

    def __init__( self ):
        self.node_name = rospy.get_name()
        self.n_subs = int(rospy.get_param('~n_subs', 2 ))
        self.publish_rate = rospy.Rate( rospy.get_param('~publish_rate', 5.0) )
        self.subs = []
        self.tracker = None
        self.ft = None # frametransformer
        self.pub = self.advertise( '~obstacles', ObstacleArray, queue_size=1 )

        # initialize sensors info
        self.sensors = []
        for i in range(self.n_subs):
            key = "~sensor{}".format(i) # get param key
            d = rospy.get_param( key ) # throw on fail

            # convert cost_fn string to functor
            d['cost_fn'] = { 
                'euclidean': EuclideanDistance(), 
                'hellinger': HellingerDistance(), 
                'bhattacharyya': BhattacharyyaDistance(),
                'perspective': PerspectiveDistance()
                }.get( d.get('cost_fn', 'euclidean') )
                
            if not d['cost_fn']:
                raise NotImplementedError( d['cost_fn'] )

            # set defaults if needed
            if not 'cost_fn_min' in d:
                d['cost_fn_min'] = 0.
            
            if not 'cost_fn_max' in d:
                d['cost_fn_max']= 10.

            d['id']=i # set sensor id
            
            self.sensors.append(d)
        
    def subscribe( self ):

        self.unsubscribe()

        self.ft = FrameTransformer()

        # init tracker
        self.tracker = SensorFusion()
        self.subs = [ rospy.Subscriber( '~%d/input' % i, ObstacleArray, callback=self.cb_sub, callback_args=i, queue_size=1, buff_size=2**24 ) for i in range(self.n_subs) ]

    def unsubscribe( self ):

        for sub in self.subs:
            sub.unregister()

        self.subs = []
        self.tracker = None
        self.ft = None

    def cb_sub( self, msg, sensor_idx ):
        "fuse input obstacle data"

        #t1 = rospy.Time.now()
        assert self.tracker

        # get sensor config
        sensor = self.sensors[sensor_idx]

        # if using PerspectiveDistance, convert origin (our platform) to target frame for current time
        if self.ft and isinstance( sensor['cost_fn'], PerspectiveDistance ):
            pos = self.ft.transform_pose( Pose(), dst_frame=msg.header.frame_id )  
            sensor['cost_fn'].origin = np.array( [pos.position.x,pos.position.y,pos.position.z ])

        self.tracker.update( sensor, [ create_tracked_object( obs ) for obs in msg.obstacles ] )
        #rospy.logwarn("processed in t=%s" % ( ( rospy.Time.now() - t1 ).to_sec() ) )

    def publish( self ):

        while not rospy.is_shutdown():
            try:
                out = ObstacleArray()
                out.header.stamp=rospy.Time.now()

                if self.tracker and self.tracker.tracked_groups:
                    self.tracker.cleanup()
                    for gt in self.tracker.tracked_groups:
                        if gt and gt.trackers:
                            with gt.lock:
                                obs = create_obstacle( gt )
                                if obs:
                                    out.obstacles.append( obs )

                # set output frame_id
                if len(out.obstacles) > 0:
                    out.header.frame_id=out.obstacles[0].header.frame_id

                self.pub.publish( out )

                self.publish_rate.sleep()
            except rospy.ROSInterruptException:
                return
            except:
                rospy.logerr( sys.exc_info() )

if __name__ == "__main__":

    rospy.init_node(ObstacleFusionNode.__name__)
    ObstacleFusionNode().publish()