#!/usr/bin/env python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import roslib
# roslib.load_manifest('video_publisher')
import sys
import rospy
import cv2
import os
import math
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from datetime import datetime
import threading
from matplotlib.cbook import toint
from numpy.f2py.auxfuncs import isinteger
from curses.ascii import isdigit
from head_control.msg import LookAtTarget
import os.path
from std_msgs.msg import Bool
import rospkg

last_fpsGlobal = 30
FPS = 30
DEV = 0
GUI = False
TrackIt=False
shouldDisable=False

def callback(data):
    global TrackIt
    global shouldDisable
    rospy.loginfo("I heard " + ('Start' if data.data==True else 'Stop'))
    if(not data.data and TrackIt):
        shouldDisable=True
    TrackIt=data.data
    
def printFPS():
    if not rospy.is_shutdown():
        threading.Timer(1, printFPS).start ()
        global last_fpsGlobal
        print float("{0:.2f}".format(last_fpsGlobal))

def sendImg():
    global FPS
    global DEV
    global last_fpsGlobal
    global TrackIt
    global shouldDisable
    rospy.init_node('camera_publisher_node', anonymous=True)
    image_pub = rospy.Publisher("/camera_publisher/output" + str(DEV) + "_" + str(FPS), Image, queue_size=10)
    head_pub=rospy.Publisher("/robotcontrol/headcontrol/target",LookAtTarget, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(DEV)
    cap.set(cv2.cv.CV_CAP_PROP_FPS, FPS)
    threading.Timer(1, printFPS).start ()
    rospack = rospkg.RosPack()
    basePath=rospack.get_path('launch')+"/config/vision/"
    face_cascade = cv2.CascadeClassifier(basePath+'haarcascade_frontalface_default.xml')
    rospy.Subscriber("/vision/face_tracker", Bool, callback)
    minlen = int(480 / 15.)
    maxlen = int(480 / 1.5)
    TimeOUT=15
    last_seen_time = time.time()-TimeOUT
    
    while not rospy.is_shutdown():
        start = datetime.now()
        headMsg=LookAtTarget()
        headMsg.enabled=True
        headMsg.is_relative=True
        headMsg.is_angular_data=True
        headMsg.vec.x=0
        headMsg.vec.y=0
        headMsg.vec.z=0
        headMsg.pitchEffort=0.3
        headMsg.yawEffort=0.3
        ret, frame = cap.read()
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        if GUI:
            cv2.imshow("output" + str(DEV) + "_" + str(FPS), frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        try:
            faces = face_cascade.detectMultiScale(gray,scaleFactor=1.35, minNeighbors=7, minSize=(minlen, minlen), maxSize=(maxlen, maxlen))
                
            if( len(faces)>1):
                faces=sorted(faces,key=lambda face:face[2]*face[3])
            index = 0
            for (x,y,w,h) in faces:
                RC=max(255-(index*60),1)
                cv2.rectangle(frame,(x,y),(x+w,y+h),(RC/2,RC/4,RC),2)
                MIDW=280.#To track a litte bit to right due to the fact that camera is not mounted properly
                MIDH=240.
                x_track=x+(w/2.)
                y_track=y+(h/2.)
                if(index==0):
                    x_track_normal=-1*min(x_track-MIDW,MIDW)/MIDW
                    if abs(x_track_normal)>0.5:
                        headMsg.vec.z=0.2*x_track_normal
                    else:
                        headMsg.vec.z=0.1*x_track_normal
                        
                    y_track_normal=min(y_track-MIDH,MIDH)/MIDH
                    if abs(y_track_normal)>0.5:
                        headMsg.vec.y=0.2*y_track_normal
                    else:
                        headMsg.vec.y=0.1*y_track_normal
                        
                    last_seen_time = time.time()
                index+=1
            image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError, e:
            print e
        fpsGlobal = 1000000 / (datetime.now() - start).microseconds
        last_fpsGlobal = (fpsGlobal * 0.1) + (last_fpsGlobal * 0.9)
        if TrackIt:
            head_pub.publish(headMsg)
        elapsed = time.time() - last_seen_time
        
        if elapsed > TimeOUT:
                headMsg=LookAtTarget()
                headMsg.enabled=True
                headMsg.is_relative=False
                headMsg.is_angular_data=True
                headMsg.vec.x=0
                headMsg.vec.y=0
                headMsg.vec.z=0
                headMsg.pitchEffort=0.25
                headMsg.yawEffort=0.25
                if TrackIt:
                    head_pub.publish(headMsg)
                    time.sleep(3)
                last_seen_time = time.time()
                
        if(shouldDisable):
            shouldDisable=False
            headMsg=LookAtTarget()
            headMsg.enabled=True
            headMsg.is_relative=False
            headMsg.is_angular_data=True
            headMsg.vec.x=0
            headMsg.vec.y=0
            headMsg.vec.z=0
            headMsg.pitchEffort=0.15
            headMsg.yawEffort=0.15
            head_pub.publish(headMsg)
            time.sleep(3)
            headMsg=LookAtTarget()
            headMsg.enabled=False
            headMsg.is_relative=False
            headMsg.is_angular_data=True
            headMsg.vec.x=0
            headMsg.vec.y=0
            headMsg.vec.z=0
            headMsg.pitchEffort=0.15
            headMsg.yawEffort=0.15
            head_pub.publish(headMsg)
            time.sleep(3)
if __name__ == '__main__':
    print "** Program Started! ***"
    if os.path.isfile("/dev/eyeRight"):
        path = os.readlink( "/dev/eyeRight" )
        if len(path) > 1 and isdigit(path[-1]):
            DEV=int(path[-1])
            print "/dev/eyeRight found at %i "%DEV
    try:
	sendImg()
    except rospy.ROSInterruptException:
	pass
   
