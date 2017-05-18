#!/usr/bin/env python
import roslib; roslib.load_manifest('widomX') #this step is necessary in order to use the customized message
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
#import the customized message
from widomX.msg import multi_cam
import scipy.misc
import numpy as np

count = 0


def callback(data):
    height = data.cam1.height
    width = data.cam1.width
    global count
    count += 1
    cam1_data = np.fromstring(data.cam1.data, dtype = np.uint8)
    cam2_data = np.fromstring(data.cam2.data, dtype = np.uint8)
    cam1_data = np.reshape(cam1_data, [height ,width, 3])
    cam2_data = np.reshape(cam2_data, [height ,width, 3])
    
    #import pdb; pdb.set_trace()
    cam1_file_name = "cam1" + str(count)+".jpg"
    cam2_file_name = "cam2" + str(count)+".jpg"
    scipy.misc.imsave(cam1_file_name, cam1_data)
    scipy.misc.imsave(cam2_file_name, cam2_data)
    print("saved")
if __name__ == '__main__':
    rospy.init_node('camera_test', anonymous = True)
    rospy.Subscriber('env_obs', multi_cam, callback)
    rospy.spin()