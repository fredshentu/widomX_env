#include <ax12.h>
#include <BioloidController.h>
#include "ax12.h"

BioloidController bioloid = BioloidController(1000000);

int incomingByte = 0;   // for incoming serial data
int direction = -1;
int torque = -1;
void setup() {
    TorqueEnable(2);
    TorqueEnable(3);
    Serial.begin(9600);
    delay(500);
    Serial.println("###################interactive torque test###########");
    CheckVoltage();
    Serial.println("Please enter the id of the servo you want to test");
}

void loop() {
     if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.parseInt();

                // say what you got:
                Serial.print("Servoid: ");
                Serial.println(incomingByte);
                Serial.println("Please enter the direction:");
                while (1) {
                    if (Serial.available() > 0) {
                        direction = Serial.parseInt();
                        break;
                    }
                }
                Serial.println("Direction:");
                Serial.println(direction);
                Serial.println("Please enter the magnitude of the torque");
                while (1) {
                    if (Serial.available() > 0) {
                        torque = Serial.parseInt();
                        break;
                    }
                }
                Serial.println("Torque:");
                Serial.println(torque);
                Serial.println("ENABLE TORQUE FOR .5 SECONDS");
                widowxTorque(incomingByte, direction, torque);
                //widowxTorque(incomingByte, direction, 0);
                Serial.println("test end, please enter the id of next servo");
                
        }
}






void CheckVoltage(){  
   // wait, then check the voltage (LiPO safety)
  float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  Serial.println("###########################");   
  Serial.print ("System Voltage: ");
  Serial.print (voltage);
  Serial.println (" volts.");
  if (voltage < 10.0){
    Serial.println("Voltage levels below 10v, please charge battery.");
    while(1);
  }  
  if (voltage > 10.0){
  Serial.println("Voltage levels nominal.");
  }

      Serial.println("###########################"); 
}
