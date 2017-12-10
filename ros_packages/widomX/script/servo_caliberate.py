#!/usr/bin/env python
import roslib; roslib.load_manifest('widomX')
import rospy
from std_msgs.msg import Int16
import numpy as np
import pickle
import time
import pdb
import argparse
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('name', type=str,
                        help='Servo name')
    args = parser.parse_args()
    servo_speed = None;
    def speed_cb(data):
        global servo_speed
        servo_speed = data.data
    
        
    rospy.init_node('servo_caliberate', anonymous = True)
    rospy.Subscriber('meter_speed', Int16, speed_cb)
    torque_controller = rospy.Publisher('Servo_torque', Int16, queue_size = 2)
    speed_trigger =rospy.Publisher('speed_trigger', Int16, queue_size = 2)
    set_id = rospy.Publisher('Servo_id', Int16, queue_size = 2)
    id_msg = Int16()
    torque_msg = Int16()
    speed_trig_msg = Int16()
    
    #Start tes
    print('start the test')
    id_msg = 2
    time.sleep(0.1)
    set_id.publish(id_msg)
    time.sleep(1.1)
    speed_traj = []
    for i in range(130):
        print("step {}".format(i))
        torque = i
        torque_msg = torque
        torque_controller.publish(torque_msg)
        time.sleep(0.1)
        speed_trigger.publish(speed_trig_msg)
        time.sleep(0.1)
        speed_traj.append(servo_speed)
    
    torque_controller.publish(100)
    time.sleep(0.2)
    torque_controller.publish(50)
    time.sleep(0.2)
    torque_controller.publish(0)
    with open("servo_data_{}.pickle".format(args.name), "wb") as handle:
        save_dict = {"speed":speed_traj}
        pickle.dump(save_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)