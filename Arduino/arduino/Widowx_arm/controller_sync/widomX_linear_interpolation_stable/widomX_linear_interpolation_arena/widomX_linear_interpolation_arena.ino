/*V2, whenever recive the command from computer, publish the current state to arm_state topic*/

#include <ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int16MultiArray.h"
#include <std_msgs/Empty.h>



#include "ax12.h"


void servo_pos_cb( const std_msgs::Int16MultiArray&  cmd_msg);
void servo_cb( const std_msgs::Int16MultiArray&  cmd_msg);
void servo_reset_cb (const std_msgs::Empty& cmd_msg);
void get_arm_state();

ros::NodeHandle nh;

/*8D array, first four speed, last four position*/
std_msgs::Int16MultiArray armdata;
ros::Publisher armstate("armstate", &armdata);
ros::Subscriber<std_msgs::Int16MultiArray> sub("servo_torque", servo_cb);
ros::Subscriber<std_msgs::Int16MultiArray> sub_pos("servo_position", servo_pos_cb);
ros::Subscriber<std_msgs::Empty> sub_reset("servo_reset", servo_reset_cb);



int data[8] = {0,0,0,0,0,0,0,0};

float old_command[4] = {0.0,0.0,0.0,0.0};
float new_command[4] = {0.0,0.0,0.0,0.0};
float command_step[4] = {0.0,0.0,0.0,0.0};
float tau = 0.15;
float one_tau = 1-tau;
int TORQUE_MODEL = 0;
int flag = 1;
float loop_counter = 19;

/*Def of zero torque: only when memset(old/new, 0, 16) has been called we raise this flag.
if torque is set 0 by step, this flag won't raise.
The default value of this flag is 0
*/
int zero_torque;
int time_out = 30;
int arm_state_timeout = 50;

//debug perpose
int count = 0;

/*get obs after excuting this function*/
void servo_reset_cb (const std_msgs::Empty& cmd_msg) {
    get_arm_state();
    armdata.data = data;
    armstate.publish(&armdata);
    if (TORQUE_MODEL == 0) {
        TorqueEnable(2);
        
        TorqueEnable(3);
        TORQUE_MODEL =1;
    }
    memset(old_command, 0, 16);
    memset(new_command, 0, 16);
    memset(command_step, 0, 16);
    zero_torque = 1;
    time_out = 30;
}



/*Reset the arm
 * This function has not been finalized
 * 
*/
void servo_pos_cb( const std_msgs::Int16MultiArray&  cmd_msg){
  /*Hard code part:
   * In order to avoid being trapped by the foam, lift the arm first(2830 or 1100) middle point 2048.
   * Set 3,4 torque 0, lift servo 2 and reset to a valid position(just randomly reset servo1?)
   */
    widowxTorque(3, 0, 0);
    widowxTorque(4, 0, 0);
    delay(1);
    
    TorqueModelDisable(2);
    ax12SetRegister2(1, AX_TORQUE_LIMIT_L, 300);  
    
    delay(1);
   
    //reset all torque
    memset(old_command, 0, 16);
    memset(new_command, 0, 16);
    memset(command_step, 0, 16);
    zero_torque = 1;
    time_out = 30;
    TORQUE_MODEL = 0;
    //one use the first and second command, and the second one is binary;
    int* command = cmd_msg.data;
    int joint;
    if (command[1] > 2048) {
      joint = 2830;
    } else {
      joint = 1100;
    }
    delay(2);
    
    int old_m1 = data[4];
    int old_m2 = data[5];
    for (int i = 0; i < 2000; i++) {
      if (old_m1 < command[0]) {
            old_m1 = old_m1 + 1;
        } else {
            old_m1 = old_m1 - 1;
        }
        if (old_m2 < joint) {
            old_m2 = old_m2 + 1;
        } else {
            old_m2 = old_m2 - 1;
        }
      
        SetPosition(1, old_m1);
        SetPosition(2, old_m2);
        delay(2);
    }
    //ax12SetRegister2(3, AX_TORQUE_LIMIT_L, 100);
    TorqueModelDisable(3);
    get_arm_state();
    
    int old_m3 = data[6];
    int old_m4 = data[7];
    ax12SetRegister2(4, AX_TORQUE_LIMIT_L, 200);
    for (int i = 0; i < 2000; i++) {
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
      
        SetPosition(3, old_m3);
        SetPosition(4, old_m4);
        delay(2);
    }
    
    delay(3);
    get_arm_state();
    armdata.data = data;
    armstate.publish(&armdata);
    ax12SetRegister2(2, AX_TORQUE_LIMIT_L, 1023);
    ax12SetRegister2(3, AX_TORQUE_LIMIT_L, 1023);
    delay(1);
}


void servo_cb( const std_msgs::Int16MultiArray&  cmd_msg){
    zero_torque = 0;
    time_out=30;
    get_arm_state();
    data[0] = count;
    armdata.data = data;
    armstate.publish(&armdata);

    count=0;
    
    //If Torque model is off, turn it on
    if (TORQUE_MODEL == 0) {
        TorqueEnable(2);
        
        TorqueEnable(3);
        TORQUE_MODEL =1;
    }
    
    /*interface with computer: torque can be positive and negative,
    Sign indicates the direction and the magnitude indicate the force
    -1023~1023*/
    //only need to update the new_command, old_command and calculate the step_size;

    for (int i = 0; i < 4; i ++) {
      old_command[i] = new_command[i];
      new_command[i] = cmd_msg.data[i];
      command_step[i] = (new_command[i] - old_command[i])/loop_counter;
      
    }
//    ax12SetRegister2(2, AX_TORQUE_LIMIT_L, 1023); 
//    ax12SetRegister2(3, AX_TORQUE_LIMIT_L, 1023); 
    
}

void setup(){
    widowxTorque(1, 0, 0);
    widowxTorque(2, 0, 0);
    widowxTorque(3, 0, 0);
    widowxTorque(4, 0, 0);
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(sub_pos);
    nh.subscribe(sub_reset);
    nh.advertise(armstate);
    armdata.data_length = 8;
    TORQUE_MODEL = 1;
    zero_torque = 1;
    time_out = 30;
    delay(100);
    /*
    for (int i = 0; i < 200; i ++) {
      nh.spinOnce();
      delay(30);
    }*/

}

//Note: If run the main loop too fast, the computer will not be able to connect to arduino
void loop(){
  count++;
   if(flag) {
  get_arm_state();
    SetPosition(1, data[4]);
    SetPosition(2, data[5]);
    SetPosition(3, data[6]);
    SetPosition(4, data[7]);
    flag = 0;
    TorqueEnable(2);
    TorqueEnable(3);
  }
  if (not zero_torque) {
    time_out = time_out - 1;
  }
  if (time_out <= 0){
    memset(old_command, 0, 16);
    memset(new_command, 0, 16);
    memset(command_step, 0, 16);
    zero_torque = 1;
    time_out = 30;
  }
    nh.spinOnce();
    delay(2);

      for (int i = 0; i < 4; i ++) {
        old_command[i] = old_command[i] + command_step[i];
      }

    
     if (TORQUE_MODEL) {
        if (old_command[0] >= 0) {
            widowxTorque(1, 1, old_command[0]);
        }else {
            widowxTorque(1, 0, -old_command[0]);
        }
        if (old_command[1]  >= 0) {
            widowxTorque(2, 1, old_command[1]);
        }else {
            widowxTorque(2, 0, -old_command[1]);
        }
     delay(2);
        if (old_command[2]  >= 0) {
            widowxTorque(3, 1, old_command[2]);
        }else {
            widowxTorque(3, 0, -old_command[2]);
        }

        if (old_command[3]  >= 0) {
            widowxTorque(4, 1, old_command[3]);
        }else {
            widowxTorque(4, 0, -old_command[3]);
        }

      }
      

}

void get_arm_state() {
  delay(3);
    int i = 0;
    for (i=0; i<4; i++) {
        /*Speed 0~1023, 1024~2047 two dirctions*/
        data[i] = GetSpeed(i + 1);
        data[i + 4] = GetPosition(i + 1);
        while (data[i]==-1 || data[i+4] == -1){
          delay(1);
          data[i] = GetSpeed(i + 1);
          data[i + 4] = GetPosition(i + 1);
          arm_state_timeout--;
          /*if (arm_state_timeout < 0) {
            //disable all servo
            widowxTorque(1, 1, 0);
            widowxTorque(2, 1, 0);
            widowxTorque(3, 1, 0);s
            widowxTorque(4, 1, 0);
            while (1) {
              //Please reboot the arduino
            }
            arm_state_timeout = 50;
          }*/
        }
    }
    arm_state_timeout = 50;
}
