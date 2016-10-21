#!/usr/bin/python
#Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import os
import subprocess
import sys
import re
from termcolor import colored
import optparse
import signal
import sys
def check_process(process):
  import re
  import subprocess
  
  retornoprocesso = False
  s = subprocess.Popen(["ps", "ax"],stdout=subprocess.PIPE)
  for x in s.stdout:
      if re.search(process, x):
          retornoprocesso = True

  return retornoprocesso
      
def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit()
        
if __name__ == "__main__":        
        signal.signal(signal.SIGINT, signal_handler)      
        if (check_process("robotcontrol")):
                print colored('Your robotcontrol is running. please close it first.',"red")
                sys.exit()  
        parser = optparse.OptionParser()
        
        parser.add_option('-m', '--max',type="string", dest="max_id",
            help="Maximum id for search", default="254")
        parser.add_option("-l", "--list",
                          action="store_true", dest="list", default=False,
                          help="List all servos tag.")
        parser.add_option("-t", "--show_tag",
                          action="store_true", dest="show_tag", default=False,
                          help="List all servos tag.")
        
        options, args = parser.parse_args()
        output=""
        hostN = subprocess.check_output("hostname", shell=True)
        isThisRobot=re.match('xs(\d+)',hostN)
        robotNumber =0 
        if isThisRobot:
                robotNumber=isThisRobot.group(1)
        
        if robotNumber == 1:
            servolist= [1,2,3,4,5,6,7,8,9,10,11,12,19,20]
            servoTag=["left_shank_pitch","left_thigh_pitch","left_hip_roll","left_hip_yaw","right_ankle_roll","right_shank_pitch","right_thigh_pitch","right_hip_roll","right_hip_yaw","left_ankle_roll","left_shoulder_pitch","right_shoulder_pitch","neck_yaw","head_pitch"]
        else:
            servolist= [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,101]
            servoTag=["left_shoulder_pitch",
            "right_shoulder_roll",
            "left_shoulder_roll",
            "right_elbow_pitch",
            "left_elbow_pitch",
            "right_hip_yaw",
            "left_hip_yaw",
            "right_hip_roll",
            "left_hip_roll",
            "right_hip_pitch",
            "left_hip_pitch",
            "right_knee_pitch",
            "left_knee_pitch",
            "right_ankle_pitch",
            "left_ankle_pitch",
            "right_ankle_roll",
            "left_ankle_roll",
            "head_yaw",
            "head_pitch",
            "right_shoulder_pitch"]
        
        if(options.show_tag):
            for idx, item in enumerate(servolist):
                print "Servo[" + str(item) + "] => " + servoTag[idx]
            print '-------------------------'
            print colored("Finish", "green")
            sys.exit(0)
        FNULL = open(os.devnull, 'w')
        MAXID=str(min(max(servolist),int(options.max_id)))
        command = 'dynatool --exec "dev 200;wait 300;write 24 1;wait 1000;scan '+MAXID+'" 2>&1 '
        print colored(command,"green")
        try:
                output = subprocess.check_output([os.getenv('SHELL'), '-i', '-c', ':;' + command ],stderr=FNULL)
        except:
                print('Exiting the program!')
                sys.exit()
        allServos=re.findall(r' (\d+) \| .{2,} \| .{2,}',output)
        print output
        print '-------------------------'
        missigServos=[]
        for idx,item in enumerate(servolist):
            if item <= int(options.max_id):
                    if not str(item) in allServos:
                        print colored("Missing servo " + str(item)+ "  ["+servoTag[idx]+"]","red")
                        missigServos.append(str(item))
                    else:
                        print colored("Found   servo " + str(item)+ "  ["+servoTag[idx]+"]","green")  
        if len(missigServos) < 1:
                FNULL.close()
                print "All servos are found!"
                sys.exit(0)
        if(options.list):
                FNULL.close()
                print '-------------------------'
                print colored("Finish","green")
                sys.exit(0)
        print '-------------------------'
        command = 'dynatool --exec "dev 200;wait 300;write 4 34;" 2>&1 '
        print colored(command,"green")
        try:
                subprocess.call([os.getenv('SHELL'), '-i', '-c', ':;' + command ],stderr=FNULL)
        except:
                print('Exiting the program!')
                sys.exit()
        print '-------------------------'
        command = 'dynatool --baud=57600 --exec "dev 200;wait 300;write 24 1;wait 1000;scan '+MAXID+'" 2>&1 '
        print colored(command,"green")
        try:
                output = subprocess.check_output([os.getenv('SHELL'), '-i', '-c', ':;' + command ],stderr=FNULL)
        except:
                print('Exiting the program!')
                sys.exit()
        print output
        allBServos=re.findall(r' (\d+) \| .{2,} \| .{2,}',output)
        print '-------------------------'
        if(len(allBServos) < len(missigServos)):
                print colored("***Warning some servos are not found in baud 34, so it might be they are dead or we have multiple servos with the same ID ***", "yellow")
        for item in allBServos:
            print colored("I found servo " + item+" on baud 34","green")
            for miss in missigServos:
                res = raw_input(colored(' -> Do you want me to change it['+item+'] to ' + miss + ' (y/N/b):',"cyan"))
                if res == "y":
                    print '-------------------------'
                    command = 'dynatool --baud=57600 --exec "dev 200;wait 300;write 24 1;wait 1000;dev ' +item + ';wait 300;write 3 ' + miss + ';wait 200;dev '+miss+';wait 300; write 4 1" 2>&1 '
                    print colored(command, "green")
                    try:
                            subprocess.call([os.getenv('SHELL'), '-i', '-c', ':;' + command ],stderr=FNULL)
                    except:
                            print('Exiting the program!')
                            sys.exit()
                    break
                elif res == "b":
                    break
                else:
                    res = raw_input(colored(' --> Write your desired ID (#/N/b):',"cyan"))
                    if res.isdigit():
                        print '-------------------------'
                        command = 'dynatool --baud=57600 --exec "dev 200;wait 300;write 24 1;wait 1000;dev ' +item + ';wait 300;write 3 ' + res + ';wait 200;dev ' + res + ';wait 300; write 4 1" 2>&1 '
                        print colored(command, "green")
                        try:
                                subprocess.call([os.getenv('SHELL'), '-i', '-c', ':;' + command ],stderr=FNULL)
                        except:
                                print('Exiting the program!')
                                sys.exit()
                        break
                    elif res == "b":
                        break    
        
        if len(allBServos)<1:
            print colored("No servo found on the 34 baud", "cyan")
        print '-------------------------'
        command = 'dynatool --baud=57600 --exec "dev 200;wait 300;write 4 1;" 2>&1 '
        print colored(command,"green")
        try:
                subprocess.call([os.getenv('SHELL'), '-i', '-c', ':;' + command ],stderr=FNULL)
        except:
                print('Exiting the program!')
                sys.exit()
        FNULL.close()
        print '-------------------------'
        print colored("Finish","green")
        
