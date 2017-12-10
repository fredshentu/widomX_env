/*V2, whenever recive the command from computer, publish the current state to arm_state topic*/

#include <ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int16MultiArray.h"
#include <std_msgs/Empty.h>
#include "ax12.h"


void servo_pos_cb( const std_msgs::Int16MultiArray&  cmd_msg);
void servo_cb( const std_msgs::Int16MultiArray&  cmd_msg);

ros::NodeHandle nh;

/*8D array, first four speed, last four position*/
std_msgs::Int16MultiArray armdata;
ros::Publisher armstate("armstate", &armdata);
ros::Subscriber<std_msgs::Int16MultiArray> sub("servo_torque", servo_cb);
ros::Subscriber<std_msgs::Int16MultiArray> sub_pos("servo_position", servo_pos_cb);


int M1;
int M2;
int M3;
int M4;

int data[8] = {0,0,0,0,0,0,0,0};
int TORQUE_MODEL = 0;

void servo_pos_cb( const std_msgs::Int16MultiArray&  cmd_msg){
    //disable torque control anyways, troque controller will enbale it
    TorqueModelDisable(2);
    TorqueModelDisable(3);
    ax12SetRegister2(1, AX_TORQUE_LIMIT_L, 300);
    ax12SetRegister2(4, AX_TORQUE_LIMIT_L, 300);
    delay(100);
    TORQUE_MODEL = 0;
    int* command = cmd_msg.data;
    get_arm_state();
    int old_m1 = data[4];
    int old_m2 = data[5];
    int old_m3 = data[6];
    int old_m4 = data[7];
    for (int i = 0; i < 2000; i++) {
      if (old_m1 < command[0]) {
          old_m1 = old_m1 + 1;
      } else {
          old_m1 = old_m1 - 1;
      }
      if (old_m2 < command[1]) {
          old_m2 = old_m2 + 1;
      } else {
          old_m2 = old_m2 - 1;
      }
      if (old_m3 < command[2]) {
          old_m3 = old_m3 + 1;
      } else {
          old_m3 = old_m3 - 1;
      }
      if (old_m4 < command[3]) {
          old_m4 = old_m4 + 1;
      } else {
          old_m4 = old_m4 - 1;
      }
        SetPosition(1, old_m1);
        SetPosition(2, old_m2);
        SetPosition(3, old_m3);
        SetPosition(4, old_m4);
        delay(2);
          
    }
    
    SetPosition(1, command[0]);
    SetPosition(2, command[1]);
    SetPosition(3, command[2]);
    SetPosition(4, command[3]);

    // SetPosition(1, command[0]);
    // SetPosition(2, command[1]);
    // SetPosition(3, command[2]);
    // SetPosition(4, command[3]);
    delay(1000);
    get_arm_state();
    armdata.data = data;
    armstate.publish(&armdata);
}


void servo_cb( const std_msgs::Int16MultiArray&  cmd_msg){
  
    get_arm_state();

    armdata.data = data;
    armstate.publish(&armdata);
    
    //If Torque model is off, turn it on
    if (TORQUE_MODEL == 0) {
        TorqueEnable(2);
        
        TorqueEnable(3);
        TORQUE_MODEL =1;
    }
    
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

void setup(){
    nh.initNode();
    nh.subscribe(sub);  
    nh.subscribe(sub_pos);
    TorqueEnable(2);
    TorqueEnable(3);
    nh.advertise(armstate);
    armdata.data_length = 8;
    TORQUE_MODEL = 1;
     widowxTorque(1, 0, 0);
    widowxTorque(2, 0, 0);
    widowxTorque(3, 0, 0);
    widowxTorque(4, 0, 0);
    // SetPosition(1, 3700);
    
}

void loop(){
    nh.spinOnce();
    get_arm_state();
    armdata.data = data;
        armstate.publish(&armdata);

}

void get_arm_state() {
    int i = 0;
    for (i=0; i<4; i++) {
        /*Speed 0~1023, 1024~2047 two dirctions*/
        data[i] = GetSpeed(i + 1);
        data[i + 4] = GetPosition(i + 1);
        /*
        while (data[i]==-1 || data[i+4] == -1){
          data[i] = GetSpeed(i + 1);
          data[i + 4] = GetPosition(i + 1);
        }*/
    }
}
