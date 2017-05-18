#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Int16MultiArray
import random


#Although this function called random_move, for now just move the arm repeatly
#for two different configurations to test the reliability of the arm.
class two_position():
    def __init__(self, pos1, pos2):
        rospy.init_node('random_move', anonymous=True)
        self.p1 = pos1
        self.p2 = pos2
        self.whichpos = 1
        self.pub = rospy.Publisher('servo', UInt16MultiArray)
        self.rate = rospy.Rate(10) # period 5seconds
        self.count = 0
    def publish(self):
        pos = None
        msg_to_send = UInt16MultiArray()

        while not rospy.is_shutdown():
            if self.whichpos:
                self.whichpos = 0
                pos = self.p2
            else:
                self.whichpos = 1
                pos = self.p1
            #this command will write the position along with current time to the log
            #located in ~/.ros/log
            rospy.loginfo("current pos:" + str(pos) + "  count{}".format(self.count))
            msg_to_send.data = pos
            self.pub.publish(msg_to_send)
            self.count +=1
            self.rate.sleep()

class random_command():
    def __init__(self):
        rospy.init_node('random_move', anonymous=True)
        self.pub = rospy.Publisher('servo', Int16MultiArray)
        self.rate = rospy.Rate(10) #10Hz
        self.count = 0
    def publish(self):
        pos = None
        msg_to_send = Int16MultiArray()

        while not rospy.is_shutdown():
            #this command will write the position along with current time to the log
            #located in ~/.ros/log
            a = random.randint(-200,200)
            b = random.randint(-70,70)
            c = random.randint(-70,70)
            d = random.randint(-400,400)
            msg_to_send.data = [a,b,c,d]
            self.pub.publish(msg_to_send)
            self.count += 1
            self.rate.sleep()

if __name__ == '__main__':
    #publisher =two_position([0,0,0,0],[1,2,3,4])
    publisher = random_command()
    try:
        publisher.publish()
    except rospy.ROSInterruptException:
        pass
