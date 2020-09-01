#!/usr/bin/env python
import copy
import numpy as np
import rospy, message_filters
from threading import Lock
from scipy.optimize import linear_sum_assignment
from asv_perception_common.msg import ObstacleArray
from asv_perception_common.NodeLazy import NodeLazy
from asv_perception_tracking.smstf.tracking import EuclideanDistance, HellingerDistance, BhattacharyyaDistance
from asv_perception_tracking.smstf.fusion import SensorFusion

from tracking import create_tracked_object  # tracking node

def create_obstacle( group_tracker ):
    """ create an obstacle msg from a smstf.TrackedObjectGroup """

    #   simple version:
    #       header:  latest
    #       observed_initial:  earliest
    #       label, label_probability:  highest, preserve if exists
    #       area, pose, dimensions, points:  from sensor with the lowest pose covariance 
    #       velocity:   from sensor with lowest velocity covar
    #       - input covariances are (always?) diagonal matrices with the same value down the diagonal
    #   use group.data to store last generated obstacle, useful for "sticky" attributes

    # construct a list of Obstacles from all trackers in group
    #   Obstacle message stored in data list
    obstacles = [ trk[1].data[0] for trk in group_tracker.trackers ]

    assert(len(obstacles)>0)

    result = copy.deepcopy(obstacles[0])
    result.id = group_tracker.id  # always use group id
    prev = group_tracker.data # previously generated obstacle, may be None

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
        
        # pose-based data
        if obs.pose.covariance[0] < result.pose.covariance[0]:
            result.pose = obs.pose
            result.area = obs.area
            result.dimensions = obs.dimensions
            result.points = obs.points

        # velocity-based data
        if obs.velocity.covariance[0] < result.velocity.covariance[0]:
            result.velocity = obs.velocity
    
    # restore any "sticky" attributes from previously-generated obstacle
    if prev:
        # classification:  use highest observed
        if prev.label_probability > result.label_probability:
            result.label_probability=prev.label_probability
            result.label=prev.label

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
            ~cost_fn:       [string, default='euclidean']
                            cost function for evaluating obstacle association
                            supported values:
                                - euclidean:     euclidean distance between centroids
                                - hellinger:     hellinger distance using means and covariances
                                - bhattacharyya: bhattacharyya distance using means and covariances
            ~cost_fn_min:   [float, default=0.]     minimum match value for the cost function
            ~cost_fn_max:   [float, default=10.]    maximum match value for the cost function
    """

    def __init__( self ):
        self.node_name = rospy.get_name()
        self.n_subs = int(rospy.get_param('~n_subs', 2 ))
        self.subs = []
        self.lock = Lock()
        self.pub = self.advertise( '~obstacles', ObstacleArray, queue_size=1 )
        self.tracker = None
        
    def subscribe( self ):

        self.unsubscribe()

        # init tracker
        # select cost function
        cost_fn = { 
            'euclidean': EuclideanDistance(), 
            'hellinger': HellingerDistance(), 
            'bhattacharyya': BhattacharyyaDistance() 
            }.get( rospy.get_param('~cost_fn', 'euclidean') )
            
        if cost_fn is None:
            raise NotImplementedError(rospy.get_param('~cost_fn'))

        self.tracker = SensorFusion( 
            cost_fn=cost_fn, 
            cost_fn_min=rospy.get_param('~cost_fn_min', 0. ),
            cost_fn_max=rospy.get_param('~cost_fn_max', 10. )
            )

        self.subs = [ rospy.Subscriber( '~%d/input' % i, ObstacleArray, callback=self.cb_sub, callback_args=i, buff_size=2**24 ) for i in range(self.n_subs) ]

    def unsubscribe( self ):

        for sub in self.subs:
            sub.unregister()

        self.subs = []
        self.tracker = None

    def cb_sub( self, msg, sensor_id ):
        """ fuse input obstacle data """
        
        with self.lock:  # callbacks may be concurrent
            group_trackers = self.tracker.update( sensor_id, [ create_tracked_object( obs ) for obs in msg.obstacles ] )

            # create/publish tracked obstacles
            out = ObstacleArray()
            out.header = msg.header
            out.obstacles = [ create_obstacle( group_tracker ) for group_tracker in group_trackers ]
            self.pub.publish( out )
    
if __name__ == "__main__":

    try:
        rospy.init_node(ObstacleFusionNode.__name__)
        n = ObstacleFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass