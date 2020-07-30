#!/usr/bin/env python

import copy, rospy, math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
from tf.transformations import *
from asv_perception_common.msg import Obstacle, ObstacleArray
from asv_perception_common.NodeLazy import NodeLazy

counter = 10000

def get_mag_linear( t ):
    """get magnitude of linear velocity"""
    v = t.velocity.twist.linear
    return math.sqrt( v.x*v.x + v.y*v.y + v.z*v.z )

def create_marker( t, type, lifetime ):

    global counter

    marker = Marker()
    marker.header = t.header
    marker.type = type
    marker.action = marker.ADD

    id = t.id
    if id is None or len(id) == 0:
        id = counter
        counter += 1

    marker.id = int( id )
    marker.lifetime = lifetime
    marker.ns = 'obstacle_tracking.%d' % type

    marker.color.a = 1.
    marker.color.r = marker.color.g = marker.color.b = 0.5

    marker.pose = copy.deepcopy( t.pose.pose )
    marker.scale.x = marker.scale.y = marker.scale.z = 1.   # default scale

    return marker

def create_marker_linestrip( t, lifetime ):
    ''' create a linestrip marker for the provided pointcloud, which should be a convex hull '''

    marker = create_marker( t, Marker.LINE_STRIP, lifetime )
    
    # points are relative to marker.pose.position
    centroid = marker.pose.position
    for pt in pc2.read_points( t.points, skip_nans=True ):     
        marker.points.append( Point( centroid.x - pt[0], centroid.y - pt[1], centroid.z - pt[2] ) )
    
    # connect first and last point
    if len(marker.points) > 0:
        marker.points.append(marker.points[0])
    
    return marker

def create_marker_cube( t, lifetime ):
    
    marker = create_marker( t, Marker.CUBE, lifetime )
    marker.scale = copy.deepcopy( t.dimensions )

    return marker
    
def create_marker_arrow( t, lifetime ):
    
    marker = create_marker( t, Marker.ARROW, lifetime )
    
    # scale.x, .y, .z --> arrow length, width, height
    marker.scale.x = 3. * get_mag_linear( t )  # exaggerate for visualization
    marker.scale.y = marker.scale.z = 0.5
    marker.pose.position.z = t.dimensions.z + 3.  # above text

    return marker

def create_marker_text( t, lifetime ):
    
    marker = create_marker( t, Marker.TEXT_VIEW_FACING, lifetime )
    marker.pose.position.z = t.dimensions.z + 1.5  # above cube
    marker.color.r = marker.color.g = marker.color.b = 1.
    
    # id, label, linear velocity, lifetime (s)
    marker.text = '%s [%s] |v|: %.2f, t: %.2f' % ( t.id, t.label, get_mag_linear( t ), ( t.header.stamp - t.observed_initial ).to_sec() )

    return marker

def create_obstacle_markers( t, lifetime ):
    """ create marker(s) for obstacle t """
    
    # always create convex hull
    hull = create_marker_linestrip( t, lifetime )

    result = [ hull ]#, create_marker_cube( t, lifetime ) ]

    # tracked
    if not t.id is None and len(t.id) > 0:
        result.append( create_marker_text( t, lifetime ) )
        hull.color.g = 1.

    # velocity arrow
    if get_mag_linear( t ) > 0.1:
        result.append( create_marker_arrow( t, lifetime ) )

    # classified obstacle
    if not t.label is None and len(t.label) > 0:
        hull.color.b = 1.

    return result


class VisualizationNode(NodeLazy):
    """ 
        Node for Obstacle visualization markers 

        Parameters:
            ~marker_duration_secs:  [float, default=0.5] seconds to display a marker
    
    """

    def __init__( self ):
        self.node_name = rospy.get_name()
        self.pub = self.advertise( '~markers', MarkerArray, queue_size=10 )
        self.sub = None
        self.marker_lifetime = rospy.Duration.from_sec( rospy.get_param( '~marker_duration_secs', 0.5 ) )

    def subscribe( self ):
        self.sub = rospy.Subscriber( '~input', ObstacleArray, self.cb_sub, queue_size=1, buff_size=2**24 )

    def unsubscribe( self ):
        if not self.sub is None:
            self.sub.unregister()
            self.sub = None

    def cb_sub( self, msg ):
            
        arr = MarkerArray()
        for obs in msg.obstacles:
            arr.markers.extend( create_obstacle_markers( obs, self.marker_lifetime ) )

        self.pub.publish(arr)
    
if __name__ == "__main__":

    try:
        rospy.init_node(VisualizationNode.__name__)
        n = VisualizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
