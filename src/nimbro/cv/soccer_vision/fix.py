
import rosbag
import sys

with rosbag.Bag(sys.argv[2], 'w') as outbag:
  for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
    if topic == "/image":
      msg.header.frame_id = "/camera_optical";
    outbag.write(topic, msg, t)
