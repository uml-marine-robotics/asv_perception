#!/usr/bin/env python

import rospy
from nmea_msgs.msg import Sentence
from asv_perception_common.msg import ObstacleArray
from NodeLazy import NodeLazy  # asv_perception_common

# based on https://stackoverflow.com/q/35669182/882436
def minmax_3d( pts ):
    
    if len(pts) < 1:
        return (None,None)
    
    minpt = type(pts[0])()
    maxpt = type(pts[0])()

    minpt.x = min( pts, key = lambda pt: pt.x ).x
    minpt.y = min( pts, key = lambda pt: pt.y ).y
    minpt.z = min( pts, key = lambda pt: pt.z ).z

    maxpt.x = max( pts, key = lambda pt: pt.x ).x
    maxpt.y = max( pts, key = lambda pt: pt.y ).y
    maxpt.z = max( pts, key = lambda pt: pt.z ).z

    return minpt, maxpt

class nmea_report_node(NodeLazy):

    def __init__( self ):
        self.node_name = rospy.get_name()
        self.pub = self.advertise( '~output', Sentence, queue_size=10 )
        self.sub = None

    def subscribe( self ):
        self.sub = rospy.Subscriber( '~input', ObstacleArray, self.cb_sub, queue_size=1 )

    def unsubscribe( self ):
        if not self.sub is None:
            self.sub.unregister()

    def cb_sub( self, msg ):
        
        for obs in msg.obstacles:

            minpt, maxpt = minmax_3d( obs.shape.points )  # produces minpt, maxpt

            if not minpt is None and not maxpt is None:

                s = Sentence()
                s.header = obs.header
                s.header.stamp = rospy.Time.now()  # don't preserve timestamp?

                hull = ''  # hull fragment for sentence
                # create 4 x 2d points from min/max, counter clockwise
                hull += "%f,%f" % ( minpt.x, minpt.y )
                hull += ",%f,%f" % ( maxpt.x, minpt.y )
                hull += ",%f,%f" % ( maxpt.x, maxpt.y )
                hull += ",%f,%f" % ( minpt.x, maxpt.y )

                # sec.nsec, id, 2d convex hull( x1,y1,x2,y2,...,xn,yn )*00\n
                obs_id = "0" if not obs.id else obs.id
                s.sentence = '$PYOBP,%d.%d,%s,%s*00\n' % ( s.header.stamp.to_sec(), s.header.stamp.to_nsec(), obs_id, hull )
                self.pub.publish(s)

    
if __name__ == "__main__":

    try:
        rospy.init_node(nmea_report_node.__name__)
        n = nmea_report_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass