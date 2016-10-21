#!/usr/bin/python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import os, sys
import subprocess
import time
from termcolor import colored
import re
from set_camera import updateCamera
from set_cm7X0 import updateCm7X0
from set_netUdev import updateNetUdev
sdStr1=""
sdStr2=""
uuidStr1=""
uuidStr2=""
robot=""
robotNr=0;
def getCol(col, line):
    p1 = line.find(col)
    if p1<0 : return ""
    p2 = p1 + len(col) + 1
    p3 = line.find('"',p2+1)
    return line[p2+1:p3]

if __name__ == '__main__':
    if not os.geteuid()==0:
            print colored("You must be root to run this application.","red")
            os._exit(-1)

    hostN = subprocess.check_output("sudo hostname", shell=True)
    isThisRobot=re.match('xs(\d+)',hostN)
    if not isThisRobot:
        print colored("You can not use this script unless you are running it on the robot (hostname=xs#).","red")
        os._exit(-1)
    if len(sys.argv) == 2:
        robot = sys.argv[1]    
    else:
        print colored("Please specify the robot name.","red")
        sys.exit()
    if("xs" not in robot or not robot.replace("xs", "").isdigit()):
        print colored("Please specify the robot name. Ex: xs4","red")
        sys.exit()
    robotNr=int(robot.replace("xs", ""))
    output = subprocess.check_output("sudo /bin/lsblk -P -o FSTYPE,UUID,MOUNTPOINT,NAME", shell=True)
    data=[]
    for line in output.split(os.linesep):
        fstype = getCol("FSTYPE", line)
        if fstype=="": continue # no fs
        uuid = getCol("UUID", line)
        name = getCol("NAME", line)
        if(fstype=="ext4"):
            sdStr1=name
            uuidStr1=uuid
        elif(fstype=="swap"):
            sdStr2=name
            uuidStr2=uuid
        data.append((fstype, uuid))
    
    #print("Found "+str(len(data))+" items")
    if len(data) > 2:
        print colored("It seems that you have more than two partition! Please check it.","red")
        sys.exit()
    if(not sdStr1 or not sdStr2 or not uuidStr1 or not uuidStr2):
        print colored("It seems that you dont have one ext4 and one swap partitions.","red")
        sys.exit()
        
    print "----------Start------------"
    print " -> Update Fstab..."
    # for (uuid) in data:
    #     print("--> %s, %s"%( uuid,fstype))
    
    
    fstab="""# /etc/fstab: static file system information.
    #
    # Use 'blkid' to print the universally unique identifier for a
    # device; this may be used with UUID= as a more robust way to name devices
    # that works even if disks are added and removed. See fstab(5).
    #
    # <file system> <mount point>   <type>  <options>       <dump>  <pass>
    # / was on /dev/"""+sdStr1+""" during installation
    UUID="""+uuidStr1+""" /               ext4    errors=remount-ro 0       1
    # swap was on /dev/"""+sdStr2+""" during installation
    UUID="""+uuidStr2+""" none            swap    sw              0       0"""
    
    with open("/etc/fstab", "w") as fw:
        fw.write(fstab)
    with open("/etc/fstab","r") as fr:
        if(fr.read() == fstab):
            print colored("  -> Done.","green")
        else:
            print colored("Can not change /etc/fstab.","red")
            sys.exit()
            
    print " -> Update Hostname..."  
    os.system("sudo hostname "+robot+" > /dev/null 2>&1")
    with open("/etc/hostname", "w") as fw:
        fw.write(robot)
    with open("/etc/hostname","r") as fr:
        if(fr.read() == robot):
            print colored("  -> Done.","green")
        else:
            print colored("Can not change /etc/hostname.","red")
            sys.exit()
          
    print " -> Update RSA..."  
    os.system("sudo rm ~/.ssh/id_rsa* > /dev/null 2>&1")
    os.system("ssh-keygen -t rsa -N nimbro -f ~/.ssh/id_rsa > /dev/null 2>&1")
    time.sleep(2)
    with open("/home/nimbro/.ssh/id_rsa.pub","r") as fr:
        id_rsa=fr.read()
        if(robot in id_rsa):
            print colored(id_rsa,"cyan")
            print colored("  -> Done.","green")
        else:
            print colored("Can not update RSA.","red")
            sys.exit()
    print " -> Update Git..."         
    os.system('git config --global user.name "'+robot+'" > /dev/null 2>&1')
    result = subprocess.check_output("git config --global user.name", shell=True)
    if(robot in result):
        print colored("  -> Done.","green")
    else:
        print colored("Can not update git.","red")
        sys.exit()
                      
    print " -> Update network interfaces..."
    netStr="""# interfaces(5) file used by ifup(8) and ifdown(8)
auto lo
iface lo inet loopback

allow-nowait eth0
iface eth0 inet dhcp
auto eth0:1
iface eth0:1 inet static
    address 192.168.100.1"""+str(robotNr)+"""
    netmask 255.255.255.0"""
    
    with open("/etc/network/interfaces", "w") as fw:
        fw.write(netStr)
    with open("/etc/network/interfaces","r") as fr:
        if(fr.read() == netStr):
            print colored("  -> Done.","green")
        else:
            print colored("Can not change /etc/network/interfaces.","red")
            sys.exit()
            
    print " -> Update network hosts..."
    netHostStr="""127.0.0.1    localhost
127.0.1.1    """+robot+""" """+robot+""".local

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters"""
    
    with open("/etc/hosts", "w") as fw:
        fw.write(netHostStr)
    with open("/etc/hosts","r") as fr:
        if(fr.read() == netHostStr):
            print colored("  -> Done.","green")
        else:
            print colored("Can not change /etc/hosts.","red")
            sys.exit()
    
    print " -> Update sudo list..."
    
    groupStr=""
    groupStrNew=""
    with open("/etc/group", "r") as fr:
        groupStr=fr.read()
    
    if(not groupStr):
        print colored("Can not open /etc/group.","red")
        sys.exit()
        
    for line in groupStr.split(os.linesep):
        if("tty:x:5" in line and not "nimbro" in line):
            groupStrNew=groupStrNew+"tty:x:5:nimbro\n"
        elif("disk:x:6" in line and not "nimbro" in line):
            groupStrNew=groupStrNew+"disk:x:6:nimbro\n"
        elif("dialout:x:20" in line and not "nimbro" in line):
            groupStrNew=groupStrNew+"dialout:x:20:nimbro\n"
        elif("video:x:44" in line and not "nimbro" in line):
            groupStrNew=groupStrNew+"video:x:44:nimbro\n"
        else:
            groupStrNew=groupStrNew+line+"\n"
    
    groupStrNew=groupStrNew.replace("\n\n", "\n")
    with open("/etc/group", "w") as fw:
        fw.write(groupStrNew)
        
    with open("/etc/group","r") as fr:
        if(fr.read() == groupStrNew):
            print colored("  -> Done.","green")
        else:
            print colored("Can not change /etc/group.","red")
            sys.exit()
    
    print " -> Update grub..."
    grubStr=""
    grubStrNew=""
    with open("/etc/default/grub", "r") as fr:
        grubStr=fr.read()
    
    if(not grubStr):
        print colored("Can not open /etc/default/grub.","red")
        sys.exit()
        
    for line in grubStr.split(os.linesep):
        if("GRUB_TIMEOUT=" in line):
            grubStrNew=grubStrNew+"GRUB_TIMEOUT=1\n"
        else:
            grubStrNew=grubStrNew+line+"\n"
            
    grubStrNew=grubStrNew.replace("\n\n", "\n")
    with open("/etc/default/grub", "w") as fw:
        fw.write(grubStrNew)
        
    with open("/etc/default/grub","r") as fr:
        if(fr.read() == grubStrNew):
            os.system("sudo update-grub > /dev/null 2>&1")
            print colored("  -> Done.","green")
        else:
            print colored("Can not change /etc/default/grub.","red")
            sys.exit()
    
    updateNetUdev()

    updateCm7X0()
            
    updateCamera()
    
    print "----------Finish-----------"
    print "----------We recommend you to run dependency_check and restart again-----------"
    print "Restart in 11"
    time.sleep(1)
    for i in list(range(10)):
        j=10-i
        print colored(str(j),"red"if j<4 else "white")
        time.sleep(1)
    os.system("sudo reboot")