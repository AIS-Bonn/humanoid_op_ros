#!/usr/bin/python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import os, sys
import subprocess
import time
import optparse
from termcolor import colored
import  threading
import rospy

parser = optparse.OptionParser()

parser.add_option('-f', '--file',type="string", dest="file",
    help="file path", default="NOFILE")

options, args = parser.parse_args()
envName="filecheck"+options.file.replace(".","_")
envVal=os.path.exists(options.file)
rospy.set_param(envName, envVal)