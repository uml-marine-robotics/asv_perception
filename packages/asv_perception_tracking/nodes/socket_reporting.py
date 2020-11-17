#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import socket, json
import rospy
import sensor_msgs.point_cloud2 as pc2
from asv_perception_common.msg import ObstacleArray
from asv_perception_common.FrameTransformer import FrameTransformer

class UdpPublisher( object ):

    def __init__( self, host, port ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM )
        self.host = host
        self.port = port

    def publish( self, b ):
        ''' publish bytes '''
        if ( self.sock.sendto( b, ( self.host, self.port ) ) < len(b) ):
            rospy.logwarn('Unable to send all %d bytes of message' % len(b) )

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

            # tf transform to correct frame
            if not self.tf_frame is None and len(self.tf_frame) > 0:
                self.ft.transform_obstacle( obs, self.tf_frame )
            
            # define list of obstacle attributes to ignore (binary data, etc)
            ignored_attrs = [ 'hull2d' ]

            serialized = json.dumps( obs
                , default=lambda o: { s: getattr(o, s) for s in o.__slots__ if hasattr(o, s) and s not in ignored_attrs }
                , ensure_ascii=False )

            self.pub.publish( bytes( serialized ) )
    
if __name__ == "__main__":

    try:
        rospy.init_node(SocketReportingNode.__name__)
        n = SocketReportingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass