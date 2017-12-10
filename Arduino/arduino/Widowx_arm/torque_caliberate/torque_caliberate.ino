/*
 * Interface to caliberate servo
 */
#include <ros.h>


#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include "ax12.h"

void servo_torque_cb(const std_msgs::Int16& torque_msg);
void servo_id_cb(const std_msgs::Int16& servo_id);
void speed_feedback(const std_msgs::Int16& trigger);
ros::NodeHandle nh;
std_msgs::Int16 meter_speed;
ros::Publisher armstate("meter_speed", &meter_speed);
ros::Subscriber<std_msgs::Int16> sub("Servo_torque", servo_torque_cb);
ros::Subscriber<std_msgs::Int16> sub_id("Servo_id", servo_id_cb);
ros::Subscriber<std_msgs::Int16> sub_trig("speed_trigger", speed_feedback);
const int METER_ID = 100;
//Servo ID should be either 2 ot 3
int ServoID = -1;

void servo_id_cb(const std_msgs::Int16& servo_id) {
  ServoID = servo_id.data;
  TorqueEnable(ServoID);
  widowxTorque(ServoID, 1,0);
}

void servo_torque_cb(const std_msgs::Int16& torque_msg) {
  int torque = torque_msg.data;
    if (ServoID != -1) {
      if (torque > 0){
        widowxTorque(ServoID, 1, torque);
      }else {
        widowxTorque(ServoID, 0, -torque);
      }
    }
}

void speed_feedback(const std_msgs::Int16& trigger) {
    if (ServoID != -1) {
    meter_speed.data = get_speed(METER_ID);
    armstate.publish(&meter_speed);
  }

}

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(sub);  
  nh.subscribe(sub_id);
  nh.subscribe(sub_trig);
  nh.advertise(armstate);
  TorqueEnable(100);
  widowxTorque(100,1,0);

}

void loop() {
   nh.spinOnce();
  //Publish the speed then

}

int get_speed(int id) {
  return GetSpeed(id);
}

