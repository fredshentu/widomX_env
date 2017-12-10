#!/usr/bin/env python
import roslib; roslib.load_manifest('widomX') #this step is necessary in order to use the customized message
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
#import the customized message
from widomX.msg import multi_cam
import scipy.misc
import numpy as np
from sensor_msgs.msg import Image
import time

RESIZE_WIDTH = 128
RESIZE_HEIGHT = 128

# RESIZE_WIDTH = 128
# RESIZE_HEIGHT = 128

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

def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.299, 0.587, 0.114])
    
    
def send_latest_frame(data):
    #rospy.loginfo(rospy.get_caller_id() + "query for frame")
    #send the latest frame
    observation.cam1 = cam1_latest_data
    observation.cam2 = cam2_latest_data
    pub.publish(observation)
    
def cam2_update(data):
    height = data.height
    width = data.width
    # print(height)
    # print(width)
    image = np.fromstring(data.data, dtype = np.uint8)
    # print(image.shape)
    image = np.reshape(image, [height, width, 3])
    
    image = scipy.misc.imresize(image, [RESIZE_WIDTH, RESIZE_HEIGHT])
    image = rgb2gray(image)
    
    cam2_before_flatten[:,:,:2] = cam2_before_flatten[:,:,1:]
    cam2_before_flatten[:,:,2] = image
        
    global cam2_latest_data
    cam2_latest_data.data = list(cam2_before_flatten.flatten()) 

def cam1_update(data):
    height = data.height
    width = data.width
    # print(height)
    # print(width)
    image = np.fromstring(data.data, dtype = np.uint8)
    image = np.reshape(image, [height, width, 3])
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
    pub = rospy.Publisher('env_obs', multi_cam, queue_size=1)
    
    rospy.Subscriber("camera_trigger", Empty, send_latest_frame)
    rospy.Subscriber(cam1_name, Image, cam1_update)
    rospy.Subscriber(cam2_name, Image, cam2_update)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    camera_buffer()