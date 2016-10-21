#!/usr/bin/python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import os, sys
import subprocess
import time
from termcolor import colored

def getCol(col, line):
    p1 = line.find(col)
    if p1<0 : return ""
    p2 = p1 + len(col) + 1
    p3 = line.find('"',p2+1)
    return line[p2+1:p3]
    
def updateCamera():
    print " -> Update device rules: eye(s)..."
    try:
        result = subprocess.check_output("which v4l2ctrl", shell=True)
        if( not "v4l2ctrl" in result):
            print colored("Cannot config webcam. Please check dependencies.","red")
            sys.exit()
    except:
        print colored("Cannot config webcam. Please check dependencies.","red")
        sys.exit()
        
    with open("/tmp/logitechConfig.txt", "w") as fw:
        fw.write("""9963776:                     Brightness:128
9963777:                       Contrast:32
9963778:                     Saturation:28
9963788:White Balance Temperature, Auto:0
9963795:                           Gain:190
9963800:           Power Line Frequency:2
9963802:      White Balance Temperature:0
9963803:                      Sharpness:191
9963804:         Backlight Compensation:1
10094849:                 Exposure, Auto:1
10094850:            Exposure (Absolute):700
10094851:        Exposure, Auto Priority:0
10094856:                 Pan (Absolute):0
10094857:                Tilt (Absolute):0
168062213:                      LED1 Mode:2
168062214:                 LED1 Frequency:255
168062321:       Disable video processing:0
168062322:             Raw bits per pixel:0
""")    
    
    with open("/tmp/logitechConfig_off.txt", "w") as fw:
        fw.write("""9963776:                     Brightness:128
9963777:                       Contrast:32
9963778:                     Saturation:28
9963788:White Balance Temperature, Auto:0
9963795:                           Gain:190
9963800:           Power Line Frequency:2
9963802:      White Balance Temperature:0
9963803:                      Sharpness:191
9963804:         Backlight Compensation:1
10094849:                 Exposure, Auto:1
10094850:            Exposure (Absolute):700
10094851:        Exposure, Auto Priority:0
10094856:                 Pan (Absolute):0
10094857:                Tilt (Absolute):0
168062213:                      LED1 Mode:0
168062214:                 LED1 Frequency:1
168062321:       Disable video processing:0
168062322:             Raw bits per pixel:0
""")
    

    result = subprocess.check_output("sudo ls /dev/video*", shell=True)

    devCount=0
    for line in result.split(os.linesep):
        numberDev = line.replace("/dev/video", "")
        if(numberDev.isdigit()):
            os.system("v4l2ctrl -d /dev/video"+numberDev+" -l /tmp/logitechConfig_off.txt > /dev/null 2>&1")
            devCount=devCount+1
    
    if(devCount==0):
        print colored("Can not find any webcam.","red")
        sys.exit()
    elif(devCount>2):
        print colored("Reduce the number of webcams to two.","red")
        sys.exit()
    time.sleep(2)
    print colored("Clear /etc/udev/rules.d/25-* & /etc/udev/rules.d/26-*","green")
    os.system("sudo rm /etc/udev/rules.d/25-* > /dev/null 2>&1")
    os.system("sudo rm /etc/udev/rules.d/26-* > /dev/null 2>&1")
    for line in result.split(os.linesep):
        numberDev = line.replace("/dev/video", "")
        
        if(numberDev.isdigit()):
            os.system("v4l2ctrl -d /dev/video"+numberDev+" -l /tmp/logitechConfig.txt > /dev/null 2>&1")
            inputStr=raw_input("%s (r/l/N) " % ("Is it right eye or left eye or None? (dev="+numberDev+")")).lower()
            selectRight=False
            selectLeft=False
            if(inputStr=='r'):
                    selectRight=True
            elif(inputStr=='l'):
                    selectLeft=True
            if(not selectRight and not selectLeft):
                    print colored("Continue ...","white")
                    continue;
            result = subprocess.check_output("udevadm info -a -p $(udevadm info -q path -p /class/video4linux/video"+numberDev+")|grep ATTRS{serial}", shell=True)
            data=[]
            toSaverule=""
            itWasSuccessfull=False
            for line in result.split(os.linesep):
                serial = getCol("ATTRS{serial}=", line)
                if len(serial)==8:
                    toSaverule='SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="080a", ATTRS{serial}=="'+serial+'", SYMLINK+="eye'+("Right" if selectRight else "Left")+'"'
                    data.append(serial)
                    print "Webcam"+numberDev+" Serial = "+  serial
                    with open("/etc/udev/rules.d/"+("25" if selectRight else "26")+"-C905-webcam.rules", "w") as fw:
                        fw.write(toSaverule)
                    with open("/etc/udev/rules.d/"+("25" if selectRight else "26")+"-C905-webcam.rules", "r") as fr:
                        rule=fr.read()
                        if(rule ==toSaverule):
                            print colored("  -> Webcam "+numberDev+" as eye"+("Right" if selectRight else "Left")+" Done.","green")
                            itWasSuccessfull=True
                        else:
                            print colored("Can not update device rules: eye"+("Right" if selectRight else "Left"),"red")
                            sys.exit()
            if(not itWasSuccessfull):
                print colored("Can not update device rules (Was it a logitech?): eye"+("Right" if selectRight else "Left"),"red") 
                            
    for line in result.split(os.linesep):
        numberDev = line.replace("/dev/video", "")
        if(numberDev.isdigit()):
            os.system("v4l2ctrl -d /dev/video"+numberDev+" -l /tmp/logitechConfig_off.txt > /dev/null 2>&1")        

if __name__ == '__main__':
    if not os.geteuid()==0:
            print colored("You must be root to run this application.","red")
            os._exit(-1)
        
    print "----------Start------------"
    
    updateCamera()
 
    print "----------Finish-----------"
    exit()