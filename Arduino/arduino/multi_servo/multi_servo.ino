//Currently this code has some problem synchronizing the feedback with the computer.
//Now it is just a open loop no feed back control, simply assign the position value to servos.

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <Braccio.h> 
#include <Servo.h> 
#include <ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt16.h"
ros::NodeHandle nh;

std_msgs::UInt16 debug_msg;
//debug indicator only return the first entry of the list.
ros::Publisher debug("debug", &debug_msg);


Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

unsigned int M1;
unsigned int M2;
unsigned int M3;
unsigned int M4;
unsigned int M5;
unsigned int M6;
void servo_cb( const std_msgs::UInt16MultiArray&  cmd_msg){
    M1 = cmd_msg.data[0];
    M2 = cmd_msg.data[1];
    M3 = cmd_msg.data[2];
    M4 = cmd_msg.data[3];
    M5 = cmd_msg.data[4];
    M6 = cmd_msg.data[5];

    Braccio.ServoMovement(10,M1,M2,M3,M4,M5,M6);
    
    //publish the debug information
    //debug_msg.data = M1;
    //debug.publish(&debug_msg);
    //nh.spinOnce();
}


ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servo_cb);

void setup(){
    nh.initNode();
    nh.subscribe(sub);
    Braccio.begin();
    Braccio.ServoMovement(20,0,90,180,180,90,30);
    //nh.advertise(debug);
    
}

void loop(){
    nh.spinOnce();
    delay(1000); //50Hz
}
