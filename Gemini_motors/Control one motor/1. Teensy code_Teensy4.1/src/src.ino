//#include <SPI.h>
#include "Gemini_Teensy41.h"
#include <FlexCAN_T4.h>
#include <Arduino.h>

// CAN_message_t msgR;
/*Canbus Setup*/
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;
/*Canbus Setup*/

uint32_t ID_offset = 0x140;
uint32_t Motor_ID1 = 1; // Motor Can Bus ID, 1，2，3，4
int CAN_ID = 3;         // CAN port from Teensy
double Gear_ratio = 9;  //The actuator gear ratio, will enfluence actuator angle and angular velocity
double torque_constant = 2.6; // after gear
uint16_t Maxspeed_position = 500;

Gemini_Teensy41 m1(Motor_ID1, CAN_ID, Gear_ratio, Maxspeed_position);

double Fsample = 300;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
unsigned long current_time = 0;
unsigned long previous_time = 0;                                                        // used to control the controller sample rate.
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);               // used to control the teensy controller frequency

double Cur_command_1 = 0;
double Vel_command_1 = 0;
double Pos_command_1 = 0;


void setup()
{
  // put your setup code here, to run once:
  delay(100);
  Serial.begin(115200);  //used for communication with computer.

  initial_CAN();
  delay(500);

  m1.init_motor(); // Strat the CAN bus communication & Motor Control
  delay(1000);

  ControlSetup();
}

void loop()
{
  CurrentControl();
  //VelocityControl();
  //PositionControl();
}

void ControlSetup()
{
  current_time = micros();
  previous_time = current_time;
}

void CurrentControl()
{
  current_time = micros();
  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    Cur_command_1 = 2 * sin(2 * PI * current_time / 1000000) / torque_constant;
    Serial.print(Cur_command_1);
    Serial.print("  ");
    Serial.println(m1.iq_A);
    m1.send_current_command(Cur_command_1);
    m1.receive_CAN_data();

    previous_time = current_time; //reset previous control loop time
  }
}

void VelocityControl()
{
  current_time = micros();
  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    Vel_command_1 = 30 * sin(2 * PI * current_time / 2000000);
    Serial.print(Vel_command_1);
    Serial.print("  ");
    Serial.println(m1.speed_value);
    m1.send_speed_command(Vel_command_1);
    m1.receive_CAN_data();

    previous_time = current_time; //reset previous control loop time
  }
}

void PositionControl()
{
  current_time = micros();
  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    Serial.print(Pos_command_1);
    Serial.print("  ");
    Serial.println(m1.encoder/Gear_ratio);
    Pos_command_1 = 60 * sin(2 * PI * current_time / 20000000);
    m1.send_position_command(Pos_command_1);
    m1.receive_CAN_data();

    previous_time = current_time; //reset previous control loop time
  }
}


void initial_CAN()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(500);
  Serial.println("Can bus setup done...");
  delay(500);
}
