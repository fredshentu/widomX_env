#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle  nh;

std_msgs::UInt16MultiArray str_msg;
ros::Publisher chatter("chatter", &str_msg);
unsigned int hello[3] = {122,31,12};

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  str_msg.data_length = 3;
}

void loop()
{
  str_msg.data = hello;
  //str_msg.layout.dim->size = 3;
  //str_msg.layout.data_offset =0;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
