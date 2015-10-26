#!/usr/bin/env python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import roslib
import sys
import rospy
import rosnode
import cv2
from std_msgs.msg import String
import numpy as np
from datetime import datetime
import os

        
def check():
    rospy.init_node('watch_vision_node', anonymous=True)
    while not rospy.is_shutdown():
        res=rosnode.rosnode_ping("/vision_module_node")
        if not res:
            os.system("v4l2ctrl -d /dev/eyeRight -l /nimbro/share/launch/config/vision/logitechConfig_off.txt")
            exit()
            
if __name__ == '__main__':
    print "Watch vision started"
    try:
	check()
    except rospy.ROSInterruptException:
	pass
   
