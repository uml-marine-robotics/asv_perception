"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import rospy

class NodeLazy(object):
    "NodeLazy, inspired by nodelet_topic_tools::NodeletLazy"

    class SubscribeListener(rospy.SubscribeListener):

        def __init__( self, cb_subscribe, cb_unsubscribe ):
            self.cb_subscribe = cb_subscribe
            self.cb_unsubscribe = cb_unsubscribe
            self.state = dict()

        def peer_subscribe(self, topic_name, topic_publish, peer_publish):
            if not True in self.state.itervalues():
                self.state[topic_name]=True
                self.cb_subscribe()

        def peer_unsubscribe(self, topic_name, num_peers):
            if num_peers < 1: # zero subscriptions to at least 1 topic
                self.state[topic_name] = False
                if not True in self.state.itervalues():
                    self.cb_unsubscribe()

    # subscribelistener/state object
    subscribe_listener = None

    # client should override, init subscriptions
    def subscribe( self ):
        raise Exception('Base class subscribe method is invoked')
        pass

    # client should override, unsubscribe
    def unsubscribe( self ):
        raise Exception('Base class unsubscribe method is invoked')
        pass

    def advertise( self, *args, **kwargs ):

        rospy.logwarn_once("[NodeLazy] This node subscribes topics only when subscribed.")

        if self.subscribe_listener is None:
            self.subscribe_listener = self.SubscribeListener( self.subscribe, self.unsubscribe )
        kwargs['subscriber_listener'] = self.subscribe_listener
        return rospy.Publisher( *args, **kwargs )