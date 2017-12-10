#!/usr/bin/env python
import roslib; roslib.load_manifest('usb2dynamixel_widomx') #this step is necessary in order to use the customized message
import roslib; roslib.load_manifest('widomX') #this step is necessary in order to use the customized message
import rospy
from audio_common_msgs.msg import AudioData
import pickle
import time
from pyhelper_fns.vis_utils import MyAnimationMulti
import numpy as np
import time
import pickle
import argparse

def gray_rgb(image):
    return np.concatenate([np.expand_dims(image, 2), np.expand_dims(image, 2), np.expand_dims(image, 2)], axis = 2)
vis_tool = MyAnimationMulti(None, numPlots=6, isIm = np.ones(6))

if __name__ == '__main__':
    
    rospy.init_node('traj_replay_audio', anonymous = True)
    pub = rospy.Publisher('replay_audio/audio', AudioData, queue_size = 10)
    msg = AudioData()
    
    parser = argparse.ArgumentParser()
    parser.add_argument('path', type=str,
                        help='path to pickle file')
                        
    args = parser.parse_args()
    FILE_NAME = args.path
    
    with open(FILE_NAME, "rb") as handle:
        data_dict = pickle.load(handle)
    states = data_dict["state_list"]
    actions = data_dict["action_list"]
    images = data_dict["image_list"]
    audio = data_dict["audio_list"]
    print("len of states{} len of actions{} len of images{}".format(len(states), len(actions), len(images)))
    import time
    for image,sound in zip(images, audio):
        image_list = []
        print(len(sound))
        for i in range(6):
            image_list.append(gray_rgb(image[:,:,i]))
        vis_tool._display(image_list)
        # import pdb; pdb.set_trace()
        msg.data = sound
        pub.publish(msg)
        time.sleep(0.09);
        
    
    