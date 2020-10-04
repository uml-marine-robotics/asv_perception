#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from broadband_radar_imager.msg import RadarSegment
from broadband_radar_imager.msg import RadarSpoke
from cv_bridge import CvBridge, CvBridgeError

import sys
import os
import numpy as np
import cv2
import math
import array
from scipy.cluster.hierarchy import complete
from docutils.nodes import header

PUB_TOPICS = ["broadband_radar/channel_0/image_raw", 
              "broadband_radar/channel_1/image_raw"]
SUB_TOPICS = ["broadband_radar/channel_0/segment",
              "broadband_radar/channel_1/segment"]

#globals
pubs = [rospy.Publisher(PUB_TOPICS[0], Image, queue_size=1000), 
        rospy.Publisher(PUB_TOPICS[1], Image, queue_size=1000)]
ptc = [] # polar(a,r) to cartesian(x,y) look-up table

# holds intensity at each x,y coordinate
intensity_maps = [np.zeros((1024,1024,3), np.uint8), 
                  np.zeros((1024,1024,3), np.uint8)]


def callback_0(segment):
  #ts = rospy.Time.now()
  process_segment(segment, 0)
  #te = rospy.Time.now()
  #rospy.logerr("time 0: " + str((te-ts).to_sec()))  
    
def callback_1(segment):
  #ts = rospy.Time.now()
  process_segment(segment, 1)
  #te = rospy.Time.now()
  #rospy.logerr("time 1: " + str((te-ts).to_sec()))  
  
def mid_range(max_range):
  # round up and devide by two to get a nice looking range circle well within max_range
  power = math.trunc(math.log10(float(max_range)))
  rounded_range = round(float(max_range)/pow(10, power)) * pow(10, power) 
  return float(rounded_range/2.0)

def process_segment(segment, index):

    global pubs, ptc
    
    spokes = segment.spokes
    for spoke in spokes:
      #rospy.loginfo("Spoke length: " + str(len(spoke.data))) 
      angle = spoke.angle
      angle_bin = int(angle*2048.0/360.0)
      arr = array.array("B", spoke.data) # convert from string of chars to array of uint8
      for bin in range(len(arr)):
        # fill in image
        #rospy.logerr("x: " + str(angle_bin) + " y: " + str(bin))
        intensity = arr[bin]
        xy_coord = ptc[angle_bin][bin]
        intensity_maps[index][xy_coord[0]][xy_coord[1]] = (0,intensity,0)
    
    max_range = spokes[0].max_range
    range_half = mid_range(max_range)
    range_quarter = range_half/2
    
    # draw circles at rounded quarter and half ranges, and at max_range
    image = np.copy(intensity_maps[index])
    cv2.circle(image, (512,512), int((range_quarter/max_range) * 512), (0,0,255))
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(image," %.0fM" % range_quarter, (512 - int((range_quarter/max_range) * 512),512), font, 0.5, (0,0,255), 1 ,cv2.LINE_AA)
    cv2.circle(image, (512,512),int((range_half/max_range) * 512), (0,0,255))
    cv2.putText(image," %.0fM" % range_half, (512 - int((range_half/max_range) * 512),512), font, 0.5, (0,0,255), 1 ,cv2.LINE_AA)
    cv2.circle(image, (512,512), 512, (0,0,255)) 
    cv2.putText(image," %.0fM" % max_range, (0,512), font, 0.5, (0,0,255), 1 ,cv2.LINE_AA)
   
    # publish 
    try:
      br = CvBridge()
      msg = br.cv2_to_imgmsg(image, 'bgr8')
      msg.header = segment.header   
      pubs[index].publish(msg)        
    except CvBridgeError as e:
      rospy.logerr("Error during cv2_to_imgmsg(): " + str(e))
    except:
      rospy.logerr("Error during image publication: " + str(sys.exc_info()[0]))  


def imager():
  
    global ptc
  
    # initialize polar-to-cartesian lookup table
    for angle_bin in range(2048):
      spoke = []
      for range_bin in range(512):
        # tricky conversion to reflect north as own ship forward 
        angle = -1*(float(angle_bin)*360.0/2048.0 + 180.0)
        x = float(range_bin) * math.cos(math.radians(angle)) 
        y = float(range_bin) * math.sin(math.radians(angle))
        x += 512 # shift -512:512 to 0:1024
        y += 512
        x = int(x)
        y = int(y)        
        spoke.append([x,y])
      ptc.append(spoke)
  
    rospy.init_node('imager', anonymous=True)
    rospy.Subscriber(SUB_TOPICS[0], RadarSegment, callback_0)
    rospy.Subscriber(SUB_TOPICS[1], RadarSegment, callback_1)

    rospy.spin()

if __name__ == '__main__':
    try:
        imager()
    except rospy.ROSInterruptException:
        pass
      