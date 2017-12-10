#!/usr/bin/env python
import roslib; roslib.load_manifest('usb2dynamixel_widomx') #this step is necessary in order to use the customized message
import roslib; roslib.load_manifest('widomX') #this step is necessary in order to use the customized message
import rospy
from audio_common_msgs.msg import AudioData
import pickle
import time


#store 10 secs audio data
if __name__ == '__main__':
    rospy.init_node("mp3_replay", anonymous = True)
    pub = rospy.Publisher('/audio/audio', AudioData, queue_size = 10)
    with open("audio_raw_mp3.pkl", 'rb') as handle:
        data = pickle.load(handle)
    array = data['audio_mp3']
    for i in array:
        msg = AudioData()
        msg.data = i
        pub.publish(msg)
        print(len(i))
        time.sleep(0.08)