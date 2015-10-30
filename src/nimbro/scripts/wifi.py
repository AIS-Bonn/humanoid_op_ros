#!/usr/bin/python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import os, sys
import subprocess
import time
import optparse
from termcolor import colored
import  threading

class Command(object):
    def __init__(self, cmd):
        self.cmd = cmd
        self.process = None

    def run(self, timeout):
        def target():
            self.process = subprocess.Popen(self.cmd, shell=True)
            self.process.communicate()

        thread = threading.Thread(target=target)
        thread.start()

        thread.join(timeout)
        if thread.is_alive():
            self.process.terminate()
            thread.join()
#         print self.process.returncode


#def colored(i,j):
#	return i
# if not root...kick out
if not os.geteuid()==0:
        print colored("You must be root to run this application.","red")
        sys.exit()

parser = optparse.OptionParser()

parser.add_option('-s', '--ssid',type="string", dest="ssid",
    help="SSID", default="modap")
parser.add_option('-p', '--pwd',type="string", dest="pwd",
    help="Pass", default="mOd4teSt")
parser.add_option('-i', '--ip',type="string", dest="ip",
    help="IP", default="")
parser.add_option('-m', '--mask',type="string", dest="mask",
    help="Subnetmask", default="")
parser.add_option("-l", "--list",
                  action="store_true", dest="listW", default=False,
                  help="List avalible networks")
parser.add_option("-b", "--block",
                  action="store_true", dest="block", default=False,
                  help="Block wireless")

options, args = parser.parse_args()

print 'SSID: ', options.ssid
print 'Pass: ', options.pwd
if not options.ip:
	print 'IP: DHCP'
else:
	print 'IP: ' , options.ip
if not options.mask:
        print 'Mask: DHCP'
else:
        print 'Mask: ' , options.mask

if options.block:
	print " -> Block wifi..."
	os.system("sudo rfkill block wifi")
	print colored("  -> Done.","green")
	sys.exit()

print "----------Start------------"
print " -> Unblock wifi..."
os.system("sudo rfkill unblock wifi")
output = subprocess.check_output("sudo rfkill list", shell=True)
# time.sleep(1)
if "Soft blocked: no" in output:
	print colored("  -> Done.","green")
else:
        print colored("  -> Failed to unblock.","red")
        sys.exit()
# time.sleep(1)


# print " -> Add wifi to list..."
# os.system("sudo ifconfig wlan0 up")
# output = subprocess.check_output("sudo ifconfig", shell=True)
# if "wlan0" in output:
# 	print colored("  -> Done.","green")
# else:
# 	print colored("  -> Failed to add.","red")
#         sys.exit()
# time.sleep(1)

if options.listW:
        print colored("list of avalible networks:","magenta")
        listW = subprocess.check_output("sudo iwlist wlan0 scan|grep SSID", shell=True)
        listW=listW.replace('"',"")
        listW=listW.replace(' ',"")
        listW=listW.replace('ESSID:'," -> ")
        print colored(listW,"cyan")
        sys.exit()

# wireless = Wireless()
# result = wireless.connect(ssid=options.ssid, password=options.pwd)
# os.system("sudo /usr/local/bin/wifi connect -a " + options.ssid)
f = open('/etc/wpa_supplicant/wpa_supplicant.conf','w')
f.write('ctrl_interface=/var/run/wpa_supplicant\n')
f.write('eapol_version=1\n')
f.write('ap_scan=1\n')
f.write('network={\n')
f.write('    ssid="'+options.ssid+'"\n')
f.write('    scan_ssid=1\n')
if len(options.pwd)>4:
    f.write('    psk="'+options.pwd+'"\n')
else:
    f.write('    key_mgmt=NONE\n')
f.write('    priority=5\n')
f.write('}\n')
f.close()
os.system("sudo service network-manager stop > /dev/null 2>&1")
os.system("sudo killall wpa_supplicant > /dev/null 2>&1")
os.system("sudo wpa_supplicant -i wlan0 -D wext -c /etc/wpa_supplicant/wpa_supplicant.conf -B > /dev/null 2>&1")
print " -> Set network Configurations..."
if not options.ip:
    os.system("ifconfig wlan0 0.0.0.0 0.0.0.0 > /dev/null")
    time.sleep(1)
    command = Command("sudo dhclient wlan0")
    command.run(timeout=15)
else:
    os.system("sudo killall dhclient > /dev/null 2>&1")
    time.sleep(1)
    setStr = "sudo ifconfig wlan0 "+options.ip
    if options.mask :
        setStr = setStr + " netmask " + options.mask
    os.system(setStr+"> /dev/null 2>&1")
    time.sleep(4)
time.sleep(1)
try:
    result = subprocess.check_output("sudo iwgetid", shell=True)
    if options.ssid in result:
        print colored("  -> Done.","green")
    else:
        print colored("  -> Failed to connect (Check password and ssid).","red")
        os._exit(0)
except:
    print colored("  -> Failed to connect (Check password and ssid ).","red")
    os._exit(0)
print "-----------Finish-----------"

