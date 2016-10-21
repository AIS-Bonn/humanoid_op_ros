#!/usr/bin/env python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import roslib
# roslib.load_manifest('video_publisher')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from datetime import datetime
import threading

last_fpsGlobal = 30
FPS = 30
DEV = 0
GUI = False

def printFPS():
    if not rospy.is_shutdown():
    	threading.Timer(1, printFPS).start ()
    	global last_fpsGlobal
    	print float("{0:.2f}".format(last_fpsGlobal))

def sendImg():
    global FPS
    global DEV
    global last_fpsGlobal
    rospy.init_node('camera_publisher_node', anonymous=True)
    image_pub = rospy.Publisher("/camera_publisher/output" + str(DEV) + "_" + str(FPS), Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(DEV)
    cap.set(cv2.cv.CV_CAP_PROP_FPS, FPS)
    threading.Timer(1, printFPS).start ()
    while not rospy.is_shutdown():
        start = datetime.now()	
    	# Capture frame-by-frame
    	ret, frame = cap.read()
        fpsGlobal = 1000000 / (datetime.now() - start).microseconds
        last_fpsGlobal = (fpsGlobal * 0.1) + (last_fpsGlobal * 0.9)
        if GUI:
            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            cv2.imshow("output" + str(DEV) + "_" + str(FPS), frame)
    	    if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        except:
            pass

if __name__ == '__main__':
    print "** Program Started! ***"
    if len(sys.argv) >= 4:
        GUI = bool(sys.argv[3])
        FPS = int(sys.argv[2])
        DEV = int(sys.argv[1])     
    elif len(sys.argv) >= 3:
        FPS = int(sys.argv[2])
        DEV = int(sys.argv[1]) 	
    elif len(sys.argv) >= 2:
	       DEV = int(sys.argv[1])
    try:
	sendImg()
    except rospy.ROSInterruptException:
	pass
   
