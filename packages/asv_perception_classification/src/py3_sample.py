#!/usr/bin/env python
# Adapted from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

import rospy
from std_msgs.msg import String

class talker:
    def __init__(self):
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
    
    def talk(self):
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        self.pub.publish(hello_str)
    
if __name__ == '__main__':
    try:
        t = talker()
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            t.talk()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
