#!/usr/bin/python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import os
import commands
import threading
import copy
import web
import shutil
import time
import re
import base64
from threading import Thread
import subprocess
import pexpect
import roslib
import rospy
import re
import datetime
from cv_bridge import CvBridge, CvBridgeError
from motion_player.srv import *
import cv2
import math
import motion_player
from motion_player.srv._StreamMotion import StreamMotion

if __name__ == "__main__":
	rospy.wait_for_service('/motion_player/update')
	try:
		motionUpdate = rospy.ServiceProxy('/motion_player/update',StreamMotion)
		resp1 = motionUpdate("kick_right",'''header:
  name: kick_right
  preState: standing
  playState: shaking_hand
  postState: standing
motion:
  - frameName: Frame 0
    duration: 1
    support: 1 0
    joints:
      left_hip_yaw:
        position: 0
        effort: 1
        velocity: 0
      left_hip_roll:
        position: 0
        effort: 1
        velocity: 0
      left_hip_pitch:
        position: 0
        effort: 1
        velocity: 0
      left_knee_pitch:
        position: 0
        effort: 1
        velocity: 0
      left_ankle_pitch:
        position: 0
        effort: 1
        velocity: 0
      left_ankle_roll:
        position: 0
        effort: 1
        velocity: 0
      left_shoulder_pitch:
        position: 0
        effort: 1
        velocity: 0
      left_shoulder_roll:
        position: 0
        effort: 1
        velocity: 0
      left_elbow_pitch:
        position: 0
        effort: 1
        velocity: 0
      neck_yaw:
        position: 0
        effort: 1
        velocity: 0
      head_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_hip_yaw:
        position: 0
        effort: 1
        velocity: 0
      right_hip_roll:
        position: 0
        effort: 1
        velocity: 0
      right_hip_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_knee_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_ankle_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_ankle_roll:
        position: 0
        effort: 1
        velocity: 0
      right_shoulder_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_shoulder_roll:
        position: 0
        effort: 1
        velocity: 0
      right_elbow_pitch:
        position: 0
        effort: 1
        velocity: 0
  - frameName: Frame 1
    duration: 20
    support: 0.5 0.5
    joints:
      left_hip_yaw:
        position: 0
        effort: 1
        velocity: 0
      left_hip_roll:
        position: 0
        effort: 1
        velocity: 0
      left_hip_pitch:
        position: 0
        effort: 1
        velocity: 0
        gain_select: 2
        d_gain: 1
      left_knee_pitch:
        position: 0
        effort: 1
        velocity: 0
      left_ankle_pitch:
        position: 0
        effort: 1
        velocity: 0
      left_ankle_roll:
        position: 0
        effort: 1
        velocity: 0
      left_shoulder_pitch:
        position: 0
        effort: 1
        velocity: 0
      left_shoulder_roll:
        position: 0
        effort: 1
        velocity: 0
      left_elbow_pitch:
        position: 0
        effort: 1
        velocity: 0
      neck_yaw:
        position: 0
        effort: 1
        velocity: 0
      head_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_hip_yaw:
        position: 0
        effort: 1
        velocity: 0
      right_hip_roll:
        position: 0
        effort: 1
        velocity: 0
      right_hip_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_knee_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_ankle_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_ankle_roll:
        position: 0
        effort: 1
        velocity: 0
      right_shoulder_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_shoulder_roll:
        position: 0
        effort: 1
        velocity: 0
      right_elbow_pitch:
        position: 0
        effort: 1
        velocity: 0
  - frameName: Frame 2
    duration: 20
    support: 0 1
    roll: 0
    joints:
      left_hip_yaw:
        position: 0
        effort: 1
        velocity: 0
      left_hip_roll:
        position: 0
        effort: 1
        velocity: 0
      left_hip_pitch:
        position: 0
        effort: 1
        velocity: 0
        gain_select: 2
        d_gain: -1
      left_knee_pitch:
        position: 0
        effort: 1
        velocity: 0
      left_ankle_pitch:
        position: 0
        effort: 1
        velocity: 0
      left_ankle_roll:
        position: 0
        effort: 1
        velocity: 0
      left_shoulder_pitch:
        position: 0
        effort: 1
        velocity: 0
      left_shoulder_roll:
        position: 0
        effort: 1
        velocity: 0
      left_elbow_pitch:
        position: 0
        effort: 1
        velocity: 0
      neck_yaw:
        position: 0
        effort: 1
        velocity: 0
      head_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_hip_yaw:
        position: 0
        effort: 1
        velocity: 0
      right_hip_roll:
        position: 0
        effort: 1
        velocity: 0
      right_hip_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_knee_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_ankle_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_ankle_roll:
        position: 0
        effort: 1
        velocity: 0
      right_shoulder_pitch:
        position: 0
        effort: 1
        velocity: 0
      right_shoulder_roll:
        position: 0
        effort: 1
        velocity: 0
      right_elbow_pitch:
        position: 0
        effort: 1
        velocity: 0''',1)
		print str(resp1)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e