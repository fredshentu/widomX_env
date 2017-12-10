#!/usr/bin/env python
import roslib; roslib.load_manifest('widomX') #this step is necessary in order to use the customized message
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray
#import the customized message
from widomX.msg import multi_cam
import scipy.misc
import numpy as np
from sensor_msgs.msg import Image
import pickle
import time


RESIZE_WIDTH = 128
RESIZE_HEIGHT = 128


"""
This camera node will not only response camera_trigger, it will also save full RGB images.
Env should tell this node when episode start and when spisode ends.
"""


cam1_latest_data = Image()
cam2_latest_data = Image()
cam1_latest_data.height = RESIZE_HEIGHT
cam1_latest_data.width = RESIZE_WIDTH
cam2_latest_data.height = RESIZE_HEIGHT
cam2_latest_data.width = RESIZE_WIDTH

pub = None

cam1_name = "/camera1/usb_cam1/image_raw"
cam2_name = "/camera2/usb_cam2/image_raw"

observation = multi_cam()

#3 channel, 3 gray_scale images
cam1_before_flatten = np.zeros([RESIZE_WIDTH, RESIZE_HEIGHT, 3], dtype=np.uint8)
cam2_before_flatten = np.zeros([RESIZE_WIDTH, RESIZE_HEIGHT, 3], dtype=np.uint8)


IN_EPISODE = 0
timesteps = 0
timeout = 5
cam1_rgb_traj = []
cam2_rgb_traj = []
cam1_3frames = np.zeros([240, 320,3*3], dtype = np.uint8)
cam2_3frames = np.zeros([240, 320,3*3], dtype = np.uint8)
fileOpen = open('log.txt', 'a')
traj_index = None
def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.299, 0.587, 0.114])
def save_rgb(command):
    global IN_EPISODE
    global timesteps
    timeout = 5
    #Episode start, start recording data
    if command.data[0] == 1:
        IN_EPISODE = 1
        global cam1_rgb_traj
        global cam2_rgb_traj
        global traj_index
        traj_index = command.data[1]
        cam1_rgb_traj = []
        cam2_rgb_traj = []
        
    #which means episode end, dump data to pickle then
    if command.data[0] == 2:
        global cam1_rgb_traj
        global cam2_rgb_traj
        
        
        traj_index = command.data[1]
        IN_EPISODE = 0
        #dump to pickle
        # import pdb; pdb.set_trace()
        print("cam_node dumping, episode {}, length{}".format(traj_index, timesteps))
        if (timesteps != 601):
            print("fatal error, timestep not correct")
            fileOpen.write("camera_node FATAL ERROR,timestep not match. Traj_index = {} timestep {}\n".format(traj_index,timesteps))
        with open("HDrgb{}.pickle".format(traj_index), "wb") as handle:
            save_dict = {"cam1":cam1_rgb_traj, "cam2":cam2_rgb_traj, "timesteps": timesteps}
            pickle.dump(save_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
        
        
    timesteps = 0
def send_latest_frame(data):
    #rospy.loginfo(rospy.get_caller_id() + "query for frame")
    #send the latest frame
    observation.cam1 = cam1_latest_data
    observation.cam2 = cam2_latest_data
    pub.publish(observation)
    
    #also push data to buffer if in episode
    if IN_EPISODE:

        cam1_rgb_traj.append(cam1_3frames.copy())
        cam2_rgb_traj.append(cam2_3frames.copy())
        global timesteps 
        global timeout
        timeout = 5
        timesteps += 1

    
def cam2_update(data):
    # now = time.time()
    global timeout
    global IN_EPISODE
    global timesteps
    if IN_EPISODE:
        timeout -= 1
        if timeout < 0:
            print("camera_node FATAL ERROR, LOST sync")
            fileOpen.write("camera_node FATAL ERROR, timeout, LOST sync. Traj_index = {} timestep {}\n".format(traj_index,timesteps))
            timeout = 5
            IN_EPISODE = 0
            timesteps = 0

    
    height = data.height
    width = data.width
    # print(height)
    # print(width)
    image = np.fromstring(data.data, dtype = np.uint8)
    
    # print(image.shape)
    image = np.reshape(image, [height, width, 3])
    
    #keep last 3frames of HD image
    cam2_3frames[:,:,:6] = cam2_3frames[:,:,3:]
    cam2_3frames[:,:,6:] = image
    
    #resize, compress images
    image = scipy.misc.imresize(image, [RESIZE_WIDTH, RESIZE_HEIGHT])
    image = rgb2gray(image)
    
    cam2_before_flatten[:,:,:2] = cam2_before_flatten[:,:,1:]
    cam2_before_flatten[:,:,2] = image
        
    global cam2_latest_data
    cam2_latest_data.data = list(cam2_before_flatten.flatten()) 
    # print("cam2 update takes {}.sec".format(time.time()-now))

def cam1_update(data):
    height = data.height
    width = data.width
    # print(height)
    # print(width)
    image = np.fromstring(data.data, dtype = np.uint8)
    image = np.reshape(image, [height, width, 3])
    
    #keep last 3frams of HD image
    cam1_3frames[:,:,:6] = cam1_3frames[:,:,3:]
    cam1_3frames[:,:,6:] = image
    
    image = scipy.misc.imresize(image, [RESIZE_WIDTH, RESIZE_HEIGHT])
    image = rgb2gray(image)
    cam1_before_flatten[:,:,:2] = cam1_before_flatten[:,:,1:]
    cam1_before_flatten[:,:,2] = image
    global cam1_latest_data
    cam1_latest_data.data = list(cam1_before_flatten.flatten())
    
def camera_buffer():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'camera_buffer' node so that multiple camera_buffers can
    # run simultaneously.
    global pub
    rospy.init_node('camera_buffer', anonymous=True)
    pub = rospy.Publisher('env_camera', multi_cam, queue_size=1)
    
    rospy.Subscriber("camera_trigger", Empty, send_latest_frame)
    rospy.Subscriber(cam1_name, Image, cam1_update)
    rospy.Subscriber(cam2_name, Image, cam2_update)
    rospy.Subscriber("episode_sync", Int16MultiArray, save_rgb)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    camera_buffer()