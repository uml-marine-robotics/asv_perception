#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from nmea_msgs.msg import Sentence
from asv_perception_common.msg import ObstacleArray
from asv_perception_common.NodeLazy import NodeLazy
from asv_perception_common.FrameTransformer import FrameTransformer

class NMEAReportNode(NodeLazy):
    """
        Class for reporting NMEA sentences

        Subscriptions:
            ~input:  [asv_perception_common/ObstacleArray.msg]  Array of obstacles to report

        Publications:
            ~output:  [nmea_msgs/Sentence.msg]  A NMEA-formatted sentence; one per obstacle

        Parameters:
            ~convex_hull:   [bool, default=True]    If true, generates/reports a convex hull from obstacle points, if exists
            ~tf_frame:      [string, default=None]  The frame to transform obstacle data into prior to reporting

    """

    def __init__( self ):
        self.node_name = rospy.get_name()
        self.convex_hull = rospy.get_param( '~convex_hull', True )
        self.tf_frame = rospy.get_param( '~tf_frame', None )
        self.sub = None
        self.ft = None
        self.pub = self.advertise( '~output', Sentence, queue_size=100 )
        
    def subscribe( self ):

        self.ft = FrameTransformer()
        self.sub = rospy.Subscriber( '~input', ObstacleArray, self.cb_sub, queue_size=1, buff_size=2**24 )

    def unsubscribe( self ):
        if not self.sub is None:
            self.sub.unregister()
            self.sub = None
        self.ft = None

    def cb_sub( self, msg ):
        
        for obs in msg.obstacles:

            # generate 2d convex hull, birds-eye view
            # if not using convex hull (optional) from obstacle points (which may not exist)
            #   generate rectangle from centroid + dimensions

            # tf transform
            if not self.tf_frame is None and len(self.tf_frame) > 0:
                self.ft.transform_obstacle( obs, self.tf_frame )
            
            pts = []
            if self.convex_hull and not obs.hull2d is None and len(obs.hull2d.points) > 0:
                # hull2d.points are points relative to pose.position, convert to absolute in tf_frame
                centroid = obs.pose.pose.position
                pts = [ (centroid.x + pt.x, centroid.y + pt.y) for pt in obs.hull2d.points ] 
        
            if len( pts ) == 0:  # no convex hull points, create hull from obstacle dimensions
                l,w = obs.dimensions.x,obs.dimensions.y
                x,y = obs.pose.pose.position.x, obs.pose.pose.position.y

                pts = [
                    ( x-l/2., y-w/2. )
                    , ( x-l/2.,y+w/2. )
                    , ( x+l/2., y+w/2. )
                    , ( x+l/2., y-w/2. )
                ]

            s = Sentence()
            s.header = obs.header
            s.header.stamp = rospy.Time.now()  # don't preserve timestamp?

            # generate hull string fragment
            hull_frag = str()
            for pt in pts:
                hull_frag += ",%f,%f" % ( pt[0], pt[1] )

            # sec.nsec, id, 2d convex hull( x1,y1,x2,y2,...,xn,yn )*00\n
            s.sentence = '$PYOBP,%d.%d,%s%s*00\n' % ( s.header.stamp.to_sec(), s.header.stamp.to_nsec(), obs.id, hull_frag )
            self.pub.publish(s)

    
if __name__ == "__main__":

    try:
        rospy.init_node(NMEAReportNode.__name__)
        n = NMEAReportNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass