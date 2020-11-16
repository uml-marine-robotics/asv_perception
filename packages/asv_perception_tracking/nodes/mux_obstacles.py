#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import rospy
import message_filters
from asv_perception_common.msg import ObstacleArray
from asv_perception_common.NodeLazy import NodeLazy

class MuxObstaclesNode( NodeLazy ):
    """ 
        Node for multiplexing and time syncing multiple ObstacleArrays 

        Subscriptions:
            ~input[0...n]:  [asv_perception_common/ObstacleArray.msg] input ObstacleArray

        Publications:
            ~obstacles:     [asv_perception_common/ObstacleArray.msg] consolidated ObstacleArray

        Parameters:
            ~max_subs:      [int, default=3]        nbr of ~input[0...n] subscribers
    """

    def __init__( self ):
        self.node_name = rospy.get_name()
        self.max_subs = int(rospy.get_param('~max_subs', 3 ))
        self.pub = self.advertise( '~obstacles', ObstacleArray, queue_size=1 )
        self.subs = []
        self.ts = None

    def subscribe( self ):

        self.unsubscribe()
        self.subs = [ message_filters.Subscriber( '~input%d' % i, ObstacleArray, buff_size=2**24 ) for i in range(self.max_subs) ]
        self.ts = message_filters.ApproximateTimeSynchronizer( self.subs, 10, 1)
        self.ts.registerCallback( self.cb_sub )

    def unsubscribe( self ):

        for sub in self.subs:
            sub.unregister()

        self.subs = []
        self.ts = None

    def cb_sub( self, *args ):
        """  append obstaclearrays, publish as one """
        
        if len(args) < 1:
            return
        
        msg = args[0]  # use the first obstacle array, append the others to it

        for i in range( 1, len(args) ):
            msg.obstacles.extend( args[i].obstacles )

        self.pub.publish( msg )
    
if __name__ == "__main__":

    try:
        rospy.init_node(MuxObstaclesNode.__name__)
        n = MuxObstaclesNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass