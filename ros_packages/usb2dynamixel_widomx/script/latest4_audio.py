#!/usr/bin/env python
import roslib; roslib.load_manifest('usb2dynamixel_widomx')
import rospy
from dynamixel_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Empty
import numpy as np

data4step = [None, None, None, None]
pub = None

def audio_cb(data):
    global data4step
    data4step[:3] = data4step[1:]
    data4step[3] = np.fromstring(data.data, dtype = np.uint8)
    
def send_audio_data(data):
    msg = UInt8MultiArray()
    msg.data = list(np.concatenate(data4step))
    pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('latest4audio', anonymous = True)
    pub = rospy.Publisher('arm_audio', UInt8MultiArray, queue_size = 10)
    rospy.Subscriber('/audio/audio', AudioData, audio_cb) 
    rospy.Subscriber('camera_trigger', Empty, send_audio_data)
    rospy.spin()