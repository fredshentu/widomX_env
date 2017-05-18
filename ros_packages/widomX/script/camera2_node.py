#!/usr/bin/env python
import roslib; roslib.load_manifest('widomX') #this step is necessary in order to use the customized message
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
#import the customized message
from widomX.msg import multi_cam

cam1_latest_data = None
cam2_latest_data = None
pub = None

cam1_name = "/camera1/usb_cam1/image_raw"
cam2_name = "/camera2/usb_cam2/image_raw"

observation = multi_cam()

def send_latest_frame(data):
    rospy.loginfo(rospy.get_caller_id() + "query for frame")
    #send the latest frame
    observation.cam1 = cam1_latest_data
    observation.cam2 = cam2_latest_data
    pub.publish(observation)
    
def cam2_update(data):
    global cam2_latest_data
    cam2_latest_data = data
    #print ("cam2 data updated")

def cam1_update(data):
    global cam1_latest_data
    cam1_latest_data = data
    #print("cam1 data updated")
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