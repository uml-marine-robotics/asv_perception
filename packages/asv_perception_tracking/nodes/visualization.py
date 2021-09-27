#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import copy, rospy, math
from visualization_msgs.msg import Marker, MarkerArray # Standard ROS messages to be used in rviz
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2
from tf.transformations import *
from asv_perception_common.msg import Obstacle, ObstacleArray
from asv_perception_common.NodeLazy import NodeLazy

counter = 10000

def get_mag_linear( t ):
    """get magnitude of linear velocity"""
    v = t.velocity.twist.linear
    return math.sqrt( v.x**2 + v.y**2 + v.z**2 )

def create_marker( t, type, lifetime ):

    global counter

    marker = Marker()
    marker.header = t.header
    marker.type = type
    marker.action = marker.ADD

    id = t.id
    if id == 0:
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
    "create a linestrip marker for the obstacle's hull2d"

    marker = create_marker( t, Marker.LINE_STRIP, lifetime )
    
    # points are relative to marker.pose.position
    if not t.hull2d is None:
        marker.points = copy.deepcopy(t.hull2d.points)
        print("Number of points in hull2d = {0}, t.label={1}".format(len(t.hull2d.points), t.label))
    
    # connect first and last point
    if len(marker.points) > 0:
        marker.points.append(marker.points[0])

    # HACK: make convex hull points not relative to orientation, need to fix in tracking.py
    marker.pose.orientation = Quaternion()
    
    return marker

def create_marker_cube( t, lifetime ):
    
    marker = create_marker( t, Marker.CUBE, lifetime )
    marker.scale = copy.deepcopy( t.dimensions )

    return marker

def create_marker_sphere( t, lifetime, sz = Vector3(1.,1.,1.) ):
    
    marker = create_marker( t, Marker.SPHERE, lifetime )
    marker.scale = copy.deepcopy( sz )

    return marker
    
def create_marker_arrow( t, lifetime ):
    
    marker = create_marker( t, Marker.ARROW, lifetime )
    
    # scale.x, .y, .z --> arrow length, width, height
    marker.scale.x = 2. * get_mag_linear( t )  # exaggerate for visualization
    marker.scale.y = marker.scale.z = 0.5
    marker.pose.position.z = t.dimensions.z + 3.  # above text

    return marker

def create_marker_text( t, lifetime ):
    
    marker = create_marker( t, Marker.TEXT_VIEW_FACING, lifetime )
    marker.pose.position.z = t.dimensions.z + 1.5  # above cube
    marker.color.r = marker.color.g = marker.color.b = 1.
    
    # id, label, area, linear velocity, lifetime (s)
    #marker.text = '%s [%s], a: %.0f |v|: %.2f, t: %.2f' % ( t.id, t.label, t.area, get_mag_linear( t ), ( t.header.stamp - t.observed_initial ).to_sec() )

    # id, label, lifetime (s)
    marker.text = '%s - %s, t: %.2f' % ( t.id, t.label, ( t.header.stamp - t.observed_initial ).to_sec() )

    return marker

def create_obstacle_markers( t, lifetime ):
    """ create marker(s) for obstacle t """
    
    # always create convex hull
    hull = create_marker_linestrip( t, lifetime )

    result = [ hull, create_marker_sphere( t, lifetime ) ]#, create_marker_cube( t, lifetime ) ]

    # tracked
    result.append( create_marker_text( t, lifetime ) )

    # velocity arrow
    #if get_mag_linear( t ) > 0.5:
    #    result.append( create_marker_arrow( t, lifetime ) )
    #    hull.color.g = 1.

    # classified obstacle
    if not t.label is None and len(t.label) > 0:
        if t.label == "obstacle":
            hull.color.r = 1.
            hull.color.g = 0.
            hull.color.b = 0.
        else:
            hull.color.b = 1.

        print("t.label={0}, t.area={1}, t.dimensions={2}, hull.color={3}".format(t.label, t.area, t.dimensions, hull.color))
    

    return result

class VisualizationNode(NodeLazy):
    """ 
        Node for Obstacle visualization markers 

        Parameters:
            ~marker_duration_secs:  [float, default=0.5] seconds to display a marker
    
    """

    def __init__( self ):
        self.node_name = rospy.get_name()
        self.marker_lifetime = rospy.Duration.from_sec( rospy.get_param( '~marker_duration_secs', 0.5 ) )
        self.sub = None
        self.sub_ir_seg_cloud = None
        self.pub = self.advertise( '~markers', MarkerArray, queue_size=10 )
        self.sub = rospy.Subscriber( '~input', ObstacleArray, self.cb_sub, queue_size=1, buff_size=2**24 )
        #self.pub_ir = self.advertise('~ir_cloud', PointCloud2, queue_size=100)
        print("visualization node={0}".format(self.node_name))

    def subscribe( self ):
        # put subscriber in init function
        #self.sub_ir = rospy.Subscriber('~input_ir', ObstacleArray, self.cb_sub_ir_seg_cloud, queue_size=1, buff_size=2**24)
        print("Subscribe in Visualization of {0} invoked".format(self.node_name))
        self.sub = rospy.Subscriber( '~input', ObstacleArray, self.cb_sub, queue_size=1, buff_size=2**24 )

    def unsubscribe( self ):
        if not self.sub is None:
            self.sub.unregister()
            self.sub = None
        if not self.sub_ir is None:
            self.sub_ir.unregister()
            self.sub_ir = None

    def cb_sub_ir_seg_cloud(self, msg):
        if (msg is not None):
            print("Receiving IR segmentation in ObstacleArray form to show...")
            arr = MarkerArray()
            for obs in msg.obstacles:
                arr.markers.extend( create_obstacle_markers( obs, self.marker_lifetime ) )

            self.pub.publish(arr)
        else:
            print("Did not receive IR segmentation in ObstacleArray form to show...")

    def cb_sub( self, msg ):
        arr = MarkerArray()
        print("Visualization cb_sub of {0} is invoked.".format(self.node_name))
        print("Number of msg.obstacles={0}, node name={1}".format(len(msg.obstacles), self.node_name))
        for obs in msg.obstacles:
            # Creates linestrip, sphere and text
            newMarkers = create_obstacle_markers( obs, self.marker_lifetime )
            arr.markers.extend(newMarkers)
            print("arr.markers[0].points={0}, arr.markers[0].color={0}, {1}, {2}, arr.markers[2].text={3}".format(newMarkers[0].points,
                                                                        newMarkers[0].color.r,
                                                                        newMarkers[0].color.g, 
                                                                        newMarkers[0].color.b, 
                                                                        newMarkers[2].text))

        print("MarkerArray has {0} markers".format(len(arr.markers)))
        self.pub.publish(arr)
    
if __name__ == "__main__":

    try:
        rospy.init_node(VisualizationNode.__name__)
        n = VisualizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
