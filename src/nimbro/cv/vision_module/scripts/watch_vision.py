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
import rospkg
import time
import rosgraph
import socket
import errno
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

rospack = rospkg.RosPack()

def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSNodeException("remote call failed: %s"%msg)
    return val
def pingNode(node_name):
    max_count=None
    ID = '/rosnode'
    master = rosgraph.Master(ID)
    node_api = rosnode.get_api_uri(master,node_name)
    if not node_api:
        return False

    timeout = 3.

    socket.setdefaulttimeout(timeout)
    node = ServerProxy(node_api)
    lastcall = 0.
    count = 0
    acc = 0.
    try:
        while True:
            try:
                start = time.time()
                pid = _succeed(node.getPid(ID))
                end = time.time()

                dur = (end-start)*1000.
                acc += dur
                count += 1

                # 1s between pings
            except socket.error as e:
                # 3786: catch ValueError on unpack as socket.error is not always a tuple
                try:
                    # #3659
                    errnum, msg = e
                    if errnum == -2: #name/service unknown
                        p = urlparse.urlparse(node_api)
                    elif errnum == errno.ECONNREFUSED:
                        # check if node url has changed
                        new_node_api = rosnode.get_api_uri(master,node_name, skip_cache=True)
                        if not new_node_api:
                            return False
                        if new_node_api != node_api:
                            node_api = new_node_api
                            node = ServerProxy(node_api)
                            continue
                    else:
                        pass
                    return False
                except ValueError:
                    pass
            if max_count and count >= max_count:
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
            
    return True

def check():
    rospy.init_node('watch_vision_node', anonymous=True)
    while not rospy.is_shutdown():
     
        node_name="/vision_module_node"
        res = pingNode(node_name)
        if not res:
            os.system("v4l2ctrl -d /dev/eyeRight -l "+rospack.get_path('launch') +"/config/vision/logitechConfig_off.txt")
            exit()
        time.sleep(0.2)
            
if __name__ == '__main__':
    print "Watch vision started"
    try:
	check()
    except rospy.ROSInterruptException:
	pass
   
