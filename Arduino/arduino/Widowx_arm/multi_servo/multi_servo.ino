#include <ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include <BioloidController.h>
#include "ax12.h"

BioloidController bioloid = BioloidController(1000000);
ros::NodeHandle nh;

/*8D array, first four speed, last four position*/
std_msgs::Int16MultiArray armdata;
ros::Publisher armstate("armstate", &armdata);
std_msgs::Int16MultiArray debugdata;
ros::Publisher debug("debug", &debugdata);

int M1;
int M2;
int M3;
int M4;

int data[8] = {0,0,0,0,0,0,0,0};
int temp[4];

void servo_cb( const std_msgs::Int16MultiArray&  cmd_msg){
    /*interface with computer: torque can be positive and negative,
    Sign indicates the direction and the magnitude indicate the force
    -1023~1023*/
    M1 = cmd_msg.data[0];
    M2 = cmd_msg.data[1];
    M3 = cmd_msg.data[2];
    M4 = cmd_msg.data[3]; 
    temp[0] = M1;
    temp[1] = M2;
    temp[2] = M3;
    temp[3] = M4;
    debugdata.data = temp;
    debug.publish(&debugdata);
    nh.spinOnce();
    /*Apply torque to actuator*/
    if (M1 >= 0) {
        widowxTorque(1, 1, M1);
    }else {
        widowxTorque(1, 0, -M1);
    }
    
    if (M2 >= 0) {
        widowxTorque(2, 1, M2);
    }else {
        widowxTorque(2, 0, -M2);
    }
    
    if (M3 >= 0) {
        widowxTorque(3, 1, M3);
    }else {
        widowxTorque(3, 0, -M3);
    }
    
    if (M4 >= 0) {
        widowxTorque(4, 1, M4);
    }else {
        widowxTorque(4, 0, -M4);
    }
}


ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_cb);

void setup(){
    nh.initNode();
    nh.subscribe(sub);
    TorqueEnable(2);
    TorqueEnable(3);
    nh.advertise(armstate);
    nh.advertise(debug);
    armdata.data_length = 8;
    debugdata.data_length = 4;
    debugdata.data = temp;
    debug.publish(&debugdata);
    
}

void loop(){
    nh.spinOnce();
    /*get the position, speed of servos*/
    get_arm_state();
    armdata.data = data;
    armstate.publish(&armdata);
    delay(10); //40Hz
}

void get_arm_state() {
    int i = 0;
    for (i=0; i<4; i++) {
        /*Speed 0~1023, 1024~2047 two dirctions*/
        data[i] = GetSpeed(i + 1);
        data[i + 4] = GetPosition(i + 1);
    }
}
