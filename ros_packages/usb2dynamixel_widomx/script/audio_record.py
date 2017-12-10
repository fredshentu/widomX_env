#!/usr/bin/env python
import roslib; roslib.load_manifest('usb2dynamixel_widomx') #this step is necessary in order to use the customized message
import roslib; roslib.load_manifest('widomX') #this step is necessary in order to use the customized message
import rospy
from audio_common_msgs.msg import AudioData
import pickle
import time

total_data = []
def audio_cb(data):
    total_data.append(data.data)
#store 10 secs audio data
if __name__ == '__main__':
    rospy.init_node("mp3_record", anonymous = True)
    rospy.Subscriber('/audio/audio', AudioData, audio_cb)
    time.sleep(10)
    save_dict = {"audio_mp3":total_data}
    with open("audio_raw_mp3.pkl", 'wb') as handle:
        pickle.dump(save_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)