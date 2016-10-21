#!/usr/bin/python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import os, sys
import subprocess
import time
from termcolor import colored
    
def updateNetUdev():
    print " -> Update device rules: network..."
    result=False
    if os.path.isfile("/etc/udev/rules.d/70-persistent-net.rules"):
            subprocess.check_output("sudo mv /etc/udev/rules.d/70-persistent-net.rules /tmp", shell=True)
    if not os.path.isfile("/etc/udev/rules.d/70-persistent-net.rules"):
            subprocess.check_output("sudo /sbin/udevadm trigger --type=devices --action=add", shell=True)
            time.sleep(3)
            if os.path.isfile("/etc/udev/rules.d/70-persistent-net.rules"):
                    result=True
            else:
                     print colored("Cannot create 70-persistent-net.reules .","red")
    else:
             print colored("Cannot mv 70-persistent-net.reules .","red")
    if(result):
            print colored("  -> Done.","green")
    else:
            print colored("Can not update device rules: network.","red")
            sys.exit()  

if __name__ == '__main__':
    if not os.geteuid()==0:
            print colored("You must be root to run this application.","red")
            os._exit(-1)
        
    print "----------Start------------"
    
    updateNetUdev()
 
    print "----------Finish-----------"
    exit()