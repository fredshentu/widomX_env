#include <ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int16MultiArray.h"
#include <std_msgs/Empty.h>
#include <BioloidController.h>
#include "ax12.h"

BioloidController bioloid = BioloidController(1000000);
ros::NodeHandle nh;

/*8D array, first four speed, last four position*/
std_msgs::Int16MultiArray armdata;
ros::Publisher armstate("armstate", &armdata);


int M1;
int M2;
int M3;
int M4;

int data[8] = {0,0,0,0,0,0,0,0};

void servo_cb( const std_msgs::Int16MultiArray&  cmd_msg){
    /*interface with computer: torque can be positive and negative,
    Sign indicates the direction and the magnitude indicate the force
    -1023~1023*/
    M1 = cmd_msg.data[0];
    M2 = cmd_msg.data[1];
    M3 = cmd_msg.data[2];
    M4 = cmd_msg.data[3]; 
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

void data_cb(const std_msgs::Empty& toggle_msg){
    get_arm_state();
    armdata.data = data;
    armstate.publish(&armdata);
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_cb);
ros::Subscriber<std_msgs::Empty> sub_data("trigger_data", data_cb);

void setup(){
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(sub_data);
    TorqueEnable(2);
    TorqueEnable(3);
    nh.advertise(armstate);
    armdata.data_length = 8;
    
}

void loop(){
    nh.spinOnce();
    delay(1);
}

void get_arm_state() {
    int i = 0;
    for (i=0; i<4; i++) {
        /*Speed 0~1023, 1024~2047 two dirctions*/
        data[i] = GetSpeed(i + 1);
        data[i + 4] = GetPosition(i + 1);
    }
}
