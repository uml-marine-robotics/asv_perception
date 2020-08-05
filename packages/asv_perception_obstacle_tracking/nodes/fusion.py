#!/usr/bin/env python
import copy
import numpy as np
import rospy, message_filters
from threading import Lock
from scipy.optimize import linear_sum_assignment
from asv_perception_common.msg import ObstacleArray
from asv_perception_common.NodeLazy import NodeLazy

def find_if( iterable, unary_predicate ):
    """ convenience wrapper, returns first (index,value) if unary_predicate(val) is True, else None """
    return next( ( (i,val) for i, val in enumerate( iterable ) if unary_predicate( val ) ), None )

def euclidean_distance( obs0, obs1 ):
    """ return squared distance between obstacle centroids """
    p0 = obs0.pose.pose.position
    p1 = obs1.pose.pose.position
    return np.sqrt( ( p0.x - p1.x )**2 + ( p0.y-p1.y )**2 + (p0.z-p1.z)**2 )

def bhattacharyya_distance( mu0, sigma0, mu1, sigma1 ):
    """
    Compute Bhattacharyya distance between two multivariate Gaussians, each defined by mu_X (mean) & sigma_X (covariance)
    https://en.wikipedia.org/wiki/Bhattacharyya_distance#Definition
    mu_X should be a 1xN numpy array
    sigma_X should be NxN numpy array
    returns scalar
    """
    mudiff = mu1-mu0    # difference of means
    mv = (sigma0 + sigma1) / 2. # mean of variances
    t1 = 0.125 * np.dot( np.dot( np.transpose(mudiff), np.linalg.inv( mv ) ), mudiff )
    t2 = 0.5 * np.log( np.linalg.det(mv) / np.sqrt( np.linalg.det(sigma0) * np.linalg.det(sigma1) ) )
    return t1+t2

def bhattacharyya_coeff( bd ):
    """
    Compute Bhattacharyya coefficient from Bhattacharyya distance (scalar)
    https://en.wikipedia.org/wiki/Bhattacharyya_distance#Bhattacharyya_coefficient
    """
    return np.exp(-bd)

def hellinger_distance( bc ):
    """
    Compute Hellinger distance from Bhattacharyya coefficient (scalar)
    """
    return np.sqrt(1-bc)


def obstacle_hellinger_distance( obs0, obs1 ):
    """
    compute hellinger distance between two obstacles
    """

    u0 = obs0.pose.pose.position
    covar0 = obs0.pose.covariance  # 6x6
    u1 = obs1.pose.pose.position
    covar1 = obs1.pose.covariance  # 6x6
    
    pm = np.array([u0.x,u0.y])
    pv = np.array([[covar0[0],covar0[1]],[covar0[6],covar0[7]]], dtype=float)
    qm = np.array([u1.x,u1.y])
    qv = np.array([[covar1[0],covar1[1]],[covar1[6],covar1[7]]], dtype=float)

    # bd --> bc --> hellinger
    bd = bhattacharyya_distance( pm, pv, qm, qv )
    bc = bhattacharyya_coeff( bd )
    return hellinger_distance( bc )

def cost_matrix( arr0, arr1, cost_fn ):
    """ compute/return 2d numpy cost matrix """
    arr = np.ndarray((len(arr0),len(arr1)))
    for i, ival in enumerate(arr0):
        for j, jval in enumerate(arr1):
            arr[i,j]= cost_fn( ival, jval )
    return arr

def match_obstacles( obs0, obs1, min_cost, max_cost, cost_fn ):
    """ match obstacles using cost fn where min_cost <= cost <= max_cost, return list of index tuple matches """

    m = cost_matrix( obs0, obs1, cost_fn )
    row_idx, col_idx = linear_sum_assignment( m )

    result = []
    for i in range( len(col_idx) ):
        cost = m[ row_idx[i], col_idx[i] ]
        
        if cost >= min_cost and cost <= max_cost:
            result.append( ( row_idx[i], col_idx[i] ) )

            # debug
            #print('======')
            #o0 = obs0[row_idx[i]]
            #o1 = obs1[col_idx[i]]
            #print('matching {(%.2f,%.2f), c=%.2f} with {(%.2f,%.2f), c=%.2f}' % ( o0.pose.pose.position.x, o0.pose.pose.position.y, o0.pose.covariance[0], o1.pose.pose.position.x, o1.pose.pose.position.y, o1.pose.covariance[0] ) )
            #print('euclidean=%.4f, hellinger=%.4f'  % ( euclidean_distance( o0, o1 ), obstacle_hellinger_distance( o0, o1 ) ) )
            
    return result

class TrackedObstacle( object ):
    """ 
    Tracked obstacle
    """

    def __init__( self, id, detection ):

        # array of (sensor_idx, obstacle), represents the most current reading from each sensor (if detected)
        self.detections = [detection]

        # assign obstacle, id
        self._obstacle = copy.deepcopy( detection[1] )
        self._obstacle.id = id

    def update( self, detection ):
        """ add/update obstacle data from detection: ( sensor_idx, obstacle ) """

        # todo:  if we want to disassociate detection from this tracker, remove the detection, return False
        existing = find_if( self.detections, lambda x : x[0] == detection[0] )
        if existing:
            self.detections[existing[0]] = detection
        else:
            self.detections.append(detection)
        
        return True

    def get_obstacle( self ):

        # may have no detections if tracker is pending delete, but may be reassociated with a different detection
        if not self.detections:
            return self._obstacle

        # else, has at least one detection
        # update obstacle data from sensor detection(s)
        #   simple version:
        #       header:  latest
        #       observed_initial:  earliest
        #       label, label_probability:  highest, preserve if exists
        #       area, pose, dimensions, points:  from sensor with the lowest pose covariance 
        #       velocity:   from sensor with lowest velocity covar
        #       - input covariances are (always?) diagonal matrices with the same value down the diagonal

        obs = self._obstacle

        pose_det = self.detections[0][1]  # pose with lowest covar
        velocity_det = self.detections[0][1]  # velocity with lowest covar

        for _, det in self.detections:

            # header
            if det.header.stamp > obs.header.stamp:
                obs.header = det.header

            # observed_initial:  use earliest
            obs.observed_initial = min( det.observed_initial, obs.observed_initial )
            
            # label, label_prob
            if det.label_probability > obs.label_probability:
                obs.label_probability = det.label_probability
                obs.label = det.label
            
            # pose-based data
            if det.pose.covariance[0] < pose_det.pose.covariance[0]:
                pose_det = det

            # velocity-based data
            if det.velocity.covariance[0] < velocity_det.velocity.covariance[0]:
                velocity_det = det

        # update pose data
        obs.pose = pose_det.pose
        obs.area = pose_det.area
        obs.dimensions = pose_det.dimensions
        obs.points = pose_det.points
        obs.header.frame_id = pose_det.header.frame_id
        # velocity data
        obs.velocity = velocity_det.velocity

        return obs

class ObstacleFusionNode( NodeLazy ):
    """ 
        Node for tracking and fusion of obstacle data

        Subscriptions:
            ~input[0...n]:  [asv_perception_common/ObstacleArray.msg] input ObstacleArray

        Publications:
            ~obstacles:     [asv_perception_common/ObstacleArray.msg] fused ObstacleArray

        Parameters:
            ~n_subs:        [int, default=2]        nbr of ~input[0...n] subscribers
            ~cost_fn:       [string, default='euclidean']
                            cost function for evaluating obstacle association
                            supported values:
                                - euclidean:     euclidean distance between centroids
                                - hellinger:     hellinger distance using means and covariances
            ~cost_fn_min:   [float, default=0.]     minimum match value for the cost function
            ~cost_fn_max:   [float, default=10.]    maximum match value for the cost function
    """

    def __init__( self ):
        self.node_name = rospy.get_name()
        self.n_subs = int(rospy.get_param('~n_subs', 2 ))
        # select the cost fn
        cost_fn_str = rospy.get_param('~cost_fn', 'distance')
        self.cost_fn = obstacle_hellinger_distance if cost_fn_str == 'hellinger' else euclidean_distance
        self.cost_fn_min = rospy.get_param('~cost_fn_min', 0. )
        self.cost_fn_max = rospy.get_param('~cost_fn_max', 10. )
        self.subs = []
        self.tracked_obstacles = []
        self.obstacle_counter = 0
        self.lock = Lock()
        self.pub = self.advertise( '~obstacles', ObstacleArray, queue_size=1 )

    def subscribe( self ):

        self.unsubscribe()
        self.subs = [ rospy.Subscriber( '~input%d' % i, ObstacleArray, callback=self.cb_sub, callback_args=i, buff_size=2**24 ) for i in range(self.n_subs) ]

    def unsubscribe( self ):

        for sub in self.subs:
            sub.unregister()

        self.subs = []

    def cb_sub( self, sensor_dets, sensor_idx ):
        """ fuse input obstacle data """
        
        with self.lock:

            # lists of unmatched indices for tracked obstacles and sensor detections
            unmatched_tracked = copy.copy( self.tracked_obstacles )  # shallow copy
            unmatched_dets = sensor_dets.obstacles

            # get current info for each tracked obstacle from this sensor
            #  for each tracked obstacle, if it has previous data from this sensor, then update it
            #  if update successful, remove tracker and detection from unmatched lists
            for i, tracked_obstacle in enumerate( unmatched_tracked ):

                # see if tracked obstacle has a previous sensor detection for current sensor_idx
                # get index of sensor-specific detection for tracked obstacle
                tracked_det = find_if( tracked_obstacle.detections, lambda det : det[0] == sensor_idx )
                if tracked_det is None:
                    continue

                # tracked has previous data for this sensor.  update it, or remove if no longer exists
                sensor_det = find_if( unmatched_dets, lambda det : not det is None and det.id == tracked_det[1][1].id )

                if sensor_det is None:  # sensor detection no longer valid.  remove tracker data
                    tracked_obstacle.detections.pop( tracked_det[0] ) 
                elif tracked_obstacle.update( (sensor_idx, sensor_det[1] ) ):  
                    # successful update, remove from untracked
                    unmatched_tracked[i]=None
                    unmatched_dets[sensor_det[0]]=None
                # else, update failed.  leave data in unmatched

            # rebuild unmatched lists
            unmatched_tracked = [ e for e in unmatched_tracked if not e is None ]
            unmatched_dets = [ e for e in unmatched_dets if not e is None ]

            # unmatched sensor detections:  attempt to match with unmatched obstacles
            if unmatched_dets:

                # do matching
                match_indices = match_obstacles( 
                    [ trk.get_obstacle() for trk in unmatched_tracked ],
                    unmatched_dets, self.cost_fn_min, self.cost_fn_max, self.cost_fn
                    )

                if match_indices:
            
                    # attempt updates of tracked obstacles from matches
                    for match in match_indices:
                        if unmatched_tracked[match[0]].update( (sensor_idx, unmatched_dets[match[1]] ) ):
                            unmatched_tracked[match[0]] = None
                            unmatched_dets[match[1]] = None

                    # rebuild unmatched lists
                    unmatched_tracked = [ e for e in unmatched_tracked if not e is None ]
                    unmatched_dets = [ e for e in unmatched_dets if not e is None ]
                
                # remaining detections:  obstacle birth
                for det in unmatched_dets:
                    self.obstacle_counter += 1
                    self.tracked_obstacles.append( TrackedObstacle( self.obstacle_counter, ( sensor_idx, det ) ) )

            # finally, prune dead trackers
            dead_obstacles = [ i for i,tracked in enumerate(self.tracked_obstacles) if len(tracked.detections) == 0 ]
            if dead_obstacles:
                for i in dead_obstacles:
                    self.tracked_obstacles[i]=None
                self.tracked_obstacles=[trk for trk in self.tracked_obstacles if not trk is None]  #rebuild list

            # publish tracked obstacles
            msg = ObstacleArray()
            msg.header = sensor_dets.header
            msg.obstacles = [ tracked.get_obstacle() for tracked in self.tracked_obstacles ]
            self.pub.publish( msg )
    
if __name__ == "__main__":

    try:
        rospy.init_node(ObstacleFusionNode.__name__)
        n = ObstacleFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass