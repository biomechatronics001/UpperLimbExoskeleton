//#include <SPI.h>
#include "Gemini_Teensy41.h"
#include <FlexCAN_T4.h>
#include <Arduino.h>
#include "WL_IMU.h"

// CAN_message_t msgR;
/*Canbus Setup*/
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;
/*Canbus Setup*/

uint32_t ID_offset = 0x140;
uint32_t Motor_ID1 = 1; // Motor Can Bus ID, 1，2，3，4
uint32_t Motor_ID2 = 2; // Motor Can Bus ID, 1，2，3，4
int CAN_ID = 3;         // CAN port from Teensy
double Gear_ratio = 9;  //The actuator gear ratio, will enfluence actuator angle and angular velocity
double torque_constant = 2.6; // after gear
uint16_t Maxspeed_position = 500;

Gemini_Teensy41 m1(Motor_ID1, CAN_ID, Gear_ratio, Maxspeed_position);
Gemini_Teensy41 m2(Motor_ID2, CAN_ID, Gear_ratio, Maxspeed_position);
IMU imu;

double Fsample = 300;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
unsigned long current_time = 0;
unsigned long previous_time = 0;                                                        // used to control the controller sample rate.
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);               // used to control the teensy controller frequency

double Cur_command_1 = 0;

double Cur_command_2 = 0;


void setup()
{
  // put your setup code here, to run once:
  delay(100);
  Serial.begin(115200);  //used for communication with computer.

  initial_CAN();
  delay(100);

  m1.init_motor(); // Strat the CAN bus communication & Motor Control
  delay(100);

  m2.init_motor(); // Strat the CAN bus communication & Motor Control
  delay(3000);
  
  imu.INIT(); //Initialize IMU;
  delay(500);
  imu.INIT_MEAN();

  //ControlSetup();
  current_time = micros();
  previous_time = current_time;
}

void loop()
{
  CurrentControl();
}

//void ControlSetup()
//{
//  current_time = micros();
//  previous_time = current_time;
//}

void CurrentControl()
{
  imu.READ();
  //custom_wait();
  
  current_time = micros();
  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    Cur_command_1 = +0.5;//1.5 * sin(2 * PI * current_time / 1000000) / torque_constant;
    Cur_command_2 = -0.5;//-1.5 * sin(2 * PI * current_time / 1000000) / torque_constant;

    //Serial.print(m1.iq_A); Serial.print("  ");
    //Serial.print(m2.iq_A); Serial.print("  ");
    Serial.print(imu.LTx); Serial.print("  ");
    Serial.print(imu.RTx); Serial.print("  ");
    Serial.print(Cur_command_1); Serial.print("  ");
    Serial.print(Cur_command_2); Serial.print("  ");
    Serial.print(m1.motorAngle); Serial.print("  ");
    Serial.print(m2.motorAngle); Serial.print("  ");
    Serial.println();

    m1.send_current_command(Cur_command_1);
    custom_wait();
    receive_CAN_data();
    custom_wait2();

    m1.read_multi_turns_angle();
    custom_wait();
    receive_CAN_data();
    custom_wait2();
    
    m2.send_current_command(Cur_command_2);
    custom_wait();
    receive_CAN_data();
    custom_wait();
    
    m2.read_multi_turns_angle();
    custom_wait();
    receive_CAN_data();
    custom_wait();

    previous_time = current_time; //reset previous control loop time
  }
}

void receive_CAN_data()
{
  Can3.read(msgR);

  if (msgR.id == (576 + Motor_ID1))//Reply: 0x240 + ID (1~32)
  {
    m1.DataExplanation(msgR);

  }
  if (msgR.id == (576 + Motor_ID2))
  {
    m2.DataExplanation(msgR);

  }
}

void initial_CAN()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(100);
  Serial.println("Can bus setup done...");
  delay(100);
}

void custom_wait()
{
//  delay(1);
  for (int  qwe = 0; qwe < 1e2; qwe++)
    {
      for (int aaa = 0; aaa < 1e2; aaa++){}
    }
}
void custom_wait2()
{
  delay(1);
//  for (int  qwe = 0; qwe < 1e8; qwe++)
//    {
//      for (int aaa = 0; aaa < 1e8; aaa++){}
//    }
}
