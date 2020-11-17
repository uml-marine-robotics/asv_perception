#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import socket, json, io

class UdpReceiver( object ):

    def __init__( self, host, port, buf_sz=2**16 ):
        self.buf_sz = buf_sz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM )
        self.sock.bind( (host,port) )
        
    def receive( self ):
        
        while True:
            data, _ = self.sock.recvfrom( self.buf_sz )     # get data
            decoded = data.decode('utf-8')
            print( 'received data: %s' % decoded )
            
            # deserialize
            # obs = json.loads( decoded )        
            # use obstacle data
            # print( 'received obstacle id=%s, position=%s' % ( obs['id'], obs['pose']['pose'] ) )

if __name__ == "__main__":
    '''
        Sample socket_reporting listener
        Usage:  python socket_reporting_listener.py
    '''

    recv = UdpReceiver( '127.0.0.1', 5555 )
    recv.receive()
    