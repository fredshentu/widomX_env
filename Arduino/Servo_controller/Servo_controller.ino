/*A general servo controller using int16Multiarray msg to communicate
  with ROS. 
*/
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;
std_msgs::Int16MultiArray debug_msg;
ros::Publisher debug("debug", &debug_msg);

int l[5] = {1,2,3,4,5};


void messageHandler(const std_msgs::Int16MultiArray& jointPosition) {
  //int length = jointPosition.layout.dim.size;
  int* positionArray = jointPosition.data;
  
  //then assign the joint angles to the servos
  debug_msg.data = positionArray;
  debug.publish(&debug_msg);
  nh.spinOnce();
  
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("jointPosition", &messageHandler);

void setup() {
 nh.initNode();
 //nh.subscribe(sub);
 //debug perpose;
 nh.advertise(debug);
}

void loop() {
  debug_msg.data = l;
  debug_msg.layout.dim->size = 5;
  debug.publish(&debug_msg);
  nh.spinOnce();
  delay(100); 
}
