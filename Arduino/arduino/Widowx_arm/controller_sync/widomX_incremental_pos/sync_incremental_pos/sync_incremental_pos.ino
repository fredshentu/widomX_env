/*Widowx incremental pos control, small torque. Arduino check */
#include <ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"
#include <std_msgs/Empty.h>
#include "ax12.h"

//pos command will be integer.
void servo_set_pos_cb(const std_msgs::Int16MultiArray&  cmd_msg);
void set_torque_limit_cb(const std_msgs::Int16MultiArray&  cmd_msg);
// void servo_reset_cb(const std_msgs::Empty& cmd_msg);
void get_arm_pos();
void get_arm_speed();

volatile int pos[4] = {0,0,0,0};
volatile int speed[4] = {0,0,0,0};
int debug[8] = {0,0,0,0,0,0,0,0};

ros::NodeHandle nh;
std_msgs::Int16MultiArray armdata; //pos send back to computer, use to sync computer with arduino
std_msgs::Int16MultiArray armdebug;
ros::Subscriber<std_msgs::Int16MultiArray> sub("set_servo_torque", set_torque_limit_cb);
//while doing position incremental control assume that computer knows the current pos
ros::Subscriber<std_msgs::Int16MultiArray> sub_pos("arm_command", servo_set_pos_cb);
ros::Publisher armstate("armstate", &armdata);
ros::Publisher pubarmdebug("armdebug", &armdebug);
//TODO: figure out servo2,3,4's valid range
void set_torque_limit_cb(const std_msgs::Int16MultiArray&  cmd_msg) {
    get_arm_pos();
    SetPosition(1, pos[0]);
    SetPosition(2, pos[1]);
    SetPosition(3, pos[2]);
    SetPosition(4, pos[3]);
    
    ax12SetRegister2(1, AX_TORQUE_LIMIT_L, cmd_msg.data[0]);
    ax12SetRegister2(2, AX_TORQUE_LIMIT_L, cmd_msg.data[1]);
    ax12SetRegister2(3, AX_TORQUE_LIMIT_L, cmd_msg.data[2]);
    ax12SetRegister2(4, AX_TORQUE_LIMIT_L, cmd_msg.data[3]);
    delay(5);
    get_arm_pos();
    armdata.data = pos;
    armstate.publish(&armdata);
}

void servo_set_pos_cb(const std_msgs::Int16MultiArray&  cmd_msg) {
    int target_pos[4] = {0,0,0,0};
    int i = 0;
    for (i=0; i<4; i++) {
        target_pos[i] = pos[i] + cmd_msg.data[i];
    }
    //blindly set the target pos, TODO: If jerkey, add interpolation BT current pos and target pos
    i = 0;
    for (i=0; i<4; i++) {
        SetPosition(i+1, target_pos[i]);
    }
    delay(100);
    //check if stuck
    get_arm_speed();
    while (int_array_max(speed) > 0) {
        delay(2);
        get_arm_speed();
    }
    get_arm_pos();
    SetPosition(1, pos[0]);
    SetPosition(2, pos[1]);
    SetPosition(3, pos[2]);
    SetPosition(4, pos[3]);
    
    armdata.data = pos;
    armstate.publish(&armdata); //sync wt computer
}

//get arm pos, if should not get -1 in this case
void get_arm_pos() {
    int i = 0;
    for (i=0; i<4; i++) {
        pos[i] = GetPosition(i + 1);
    }
}
//the absolute value of speed
void get_arm_speed() {
    int i = 0;
    int temp;
    for (i=0; i<4; i++) {
        temp = GetSpeed(i + 1);
        if (temp > 1023) {
            speed[i] = temp - 1024;
        }else{
            speed[i] = temp;
        }
    }
}

//currently only work for 4d array
int int_array_max(int* list) {
    int i = 0;
    int max = -1;
    for (i=0; i<4; i++) {
        if (list[i] > max) {
            max = list[i];
        }
    }
    return max;
}

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(sub_pos);
    nh.advertise(armstate);
    nh.advertise(pubarmdebug);
    armdata.data_length = 4;
    armdebug.data_length = 8;
    delay(100);
    
}

void loop() {
    nh.spinOnce();
//    get_arm_pos();
//    armdata.data = pos;
//    armstate.publish(&armdata);
    
}
