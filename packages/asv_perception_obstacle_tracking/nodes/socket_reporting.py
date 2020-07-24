#!/usr/bin/env python

import socket
import rospy
import sensor_msgs.point_cloud2 as pc2
from asv_perception_common.msg import ObstacleArray
from asv_perception_common.FrameTransformer import FrameTransformer

class UdpPublisher( object ):

    def __init__( self, host, port ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM )
        self.host = host
        self.port = port

    def publish( self, s ):
        ''' publish a utf-8 string '''
        self.sock.sendto( bytes( s ), ( self.host, self.port ) )

class SocketReportingNode( object ):
    """
        Example class for reporting obstacle data over sockets

        Subscriptions:
            ~input:  [asv_perception_common/ObstacleArray.msg]  Array of obstacles to report

        Parameters:
            ~tf_frame:      [string, default=None]  The frame to transform obstacle data into prior to reporting
            ~host_ip:       [string, default='127.0.0.1']  socket host ip
            ~port:          [int, default=5555]  socket port
            ~proto:         [string, default='udp']  socket protocol
    """

    def __init__( self ):
        self.node_name = rospy.get_name()
        self.tf_frame = rospy.get_param( '~tf_frame', None )
        self.pub = None

        host = rospy.get_param( '~host', '127.0.0.1' )
        port = rospy.get_param( '~port', 5555 )

        if rospy.get_param( '~proto', 'udp' ) == 'udp':
            self.pub = UdpPublisher( host, port )
        
        if self.pub is None:
            raise ValueError(rospy.get_param('~proto'))

        self.ft = FrameTransformer()
        self.sub = rospy.Subscriber( '~input', ObstacleArray, self.cb_sub, queue_size=1, buff_size=2**24 )

    def cb_sub( self, msg ):
        
        for obs in msg.obstacles:

            # tf transform
            if not self.tf_frame is None and len(self.tf_frame) > 0:
                self.ft.transform_obstacle( obs, self.tf_frame )
            
            l,w = obs.dimensions.x,obs.dimensions.y
            x,y = obs.pose.pose.position.x, obs.pose.pose.position.y

            pts = [
                ( x-l/2., y-w/2. )
                , ( x-l/2.,y+w/2. )
                , ( x+l/2., y+w/2. )
                , ( x+l/2., y-w/2. )
            ]

            # generate hull string fragment
            hull_frag = str()
            for pt in pts:
                hull_frag += "%f,%f" % ( pt[0], pt[1] )

            # sec.nsec, id, 2d points ( x1,y1,x2,y2,...,xn,yn )*00\n
            obs_id = "0" if not obs.id else obs.id
            s = '$PYOBP,%d.%d,%s,%s*00\n' % ( msg.header.stamp.to_sec(), msg.header.stamp.to_nsec(), obs_id, hull_frag )
            self.pub.publish( s )
            
    
if __name__ == "__main__":

    try:
        rospy.init_node(SocketReportingNode.__name__)
        n = SocketReportingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass