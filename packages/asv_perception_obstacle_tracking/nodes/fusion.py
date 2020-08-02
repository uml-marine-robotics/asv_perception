#!/usr/bin/env python

import rospy
import message_filters
from asv_perception_common.msg import ObstacleArray
from asv_perception_common.NodeLazy import NodeLazy

class ObstacleFusionNode( NodeLazy ):
    """ 
        Node for 

        Subscriptions:
            ~input[0...n]:  [asv_perception_common/ObstacleArray.msg] input ObstacleArray

        Publications:
            ~obstacles:     [asv_perception_common/ObstacleArray.msg] fused ObstacleArray

        Parameters:
            ~max_subs:      [int, default=3]        nbr of ~input[0...n] subscribers
            ~slop:          [float, default=2.0]    approximate time sync slop
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
        self.ts = message_filters.ApproximateTimeSynchronizer( self.subs, 10, rospy.get_param('~slop', 2. ) )
        self.ts.registerCallback( self.cb_sub )

    def unsubscribe( self ):

        for sub in self.subs:
            sub.unregister()

        self.subs = []
        self.ts = None

    def cb_sub( self, *args ):
        ''' perform fusion '''
        
        if len(args) < 1:
            return
        
        rospy.logwarn('Fusion:  cb_sub')
    
if __name__ == "__main__":

    try:
        rospy.init_node(ObstacleFusionNode.__name__)
        n = ObstacleFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass