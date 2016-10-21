#!/usr/bin/python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import os, sys
import subprocess
import time
from termcolor import colored

if __name__ == '__main__':
    if not os.geteuid()==0:
            print colored("You must be root to run this application.","red")
            os._exit(-1)
        
    print "----------Start------------"
    
    f = open("/etc/ssh/sshd_config","r")
    lines = f.readlines()
    f.close()
    
    f = open("/etc/ssh/sshd_config","w")
    fix=[False,False]
    for line in lines:
            if("GSSAPIAuthentication" in line):
                    f.write("GSSAPIAuthentication no\n")
                    fix[0]=True
            elif("UseDNS" in line):
                    f.write("UseDNS no\n")
                    fix[1]=True
            else:
                    f.write(line)
    if(not fix[0]):
            f.write("GSSAPIAuthentication no\n")
            fix[0]= True
    if(not fix[1]):
            f.write("UseDNS no\n")
            fix[1]= True
    f.close()
    print colored("  -> Done.","green")
    print "----------Finish-----------"
    exit()