#!/usr/bin/env python
import socket

class UdpReceiver( object ):

    def __init__( self, host, port, buf_sz=1024 ):
        self.buf_sz = buf_sz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM )
        self.sock.bind( (host,port) )
        

    def receive( self ):
        
        while True:
            data = self.sock.recvfrom( self.buf_sz )
            print("received message: %s" % data[0] )

if __name__ == "__main__":
    '''
        Sample socket_reporting listener
        Usage:  python socket_reporting_listener.py
    '''

    recv = UdpReceiver( '127.0.0.1', 5555 )
    recv.receive()
    