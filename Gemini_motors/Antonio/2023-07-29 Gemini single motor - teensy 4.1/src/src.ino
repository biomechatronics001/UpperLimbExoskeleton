//#include <SPI.h> Isra-Jason
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

Gemini_Teensy41 m1(Motor_ID1, CAN_ID, Gear_ratio);

double Fsample = 150;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
unsigned long current_time = 0;
unsigned long previous_time = 0;                                                        // used to control the controller sample rate.
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);               // used to control the teensy controller frequency

double Cur_command_1 = 0;
double Vel_command_1 = 0;
double Pos_command_1 = 0;

double position_zero1 = 0;
double position1 = 0;
double position1_prev = 0;
double error = 0;
double error_prev = 0;

double set_point = 0;

double PID_P = 0;
double PID_D = 0;

double P = 20;
double D = 2;

unsigned long delay_control2 = 2000;

void setup()
{
  // put your setup code here, to run once:
  Serial.print("Setting up, do not move end effector!");
  delay(100);
  Serial.begin(115200);  //used for communication with computer.

  initial_CAN();
  delay(500);

  m1.init_motor(); // Strat the CAN bus communication & Motor Control
  delay(1000);

  ControlSetup();

  reset_motor_angle();

  delay(2);
  Serial.print("Set up complete.");

  // Ensure end effector is centered before beginning
  // Update the bounds to match actual encoder position for center
//  while (position_zero1 < -5 || position_zero1 > 5) {
//    m1.read_multi_turns_angle();
//    m1.receive_CAN_data();
//    delay(2);
//    position_zero1 = m1.motorAngle;
//    Serial.print("Pos1:");
//    Serial.println(position_zero1);
//    Serial.println("End effector is not centered");
//    m1.send_current_command(Cur_command_1);
//    m1.receive_CAN_data();
//    delay(2);
//  }
//  Serial.println("End effector centered. Connection Initialized.");
  
}

void loop()
{
  //CurrentControl();
  //VelocityControl();

  // // Check that motor position does not have a sudden jump of more than 44 degrees
  // if(m1.motorAngle > position1 + 44 || m1.motorAngle < position1 - 44){
  //   exit(0);
  // }

  CurrentControl();
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

    Cur_command_1 = 1;//;*sin(2*PI*current_time/1e6);

    m1.send_current_command(Cur_command_1);
    m1.receive_CAN_data();
    delay(2);
    m1.read_multi_turns_angle(); //read angle and angle velocity
    m1.receive_CAN_data();
    delay(2);
    
    Serial.print(Cur_command_1); Serial.print("   ");
    Serial.print(m1.motorAngle); Serial.print("   ");
    Serial.println();
    
    previous_time = current_time;
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

void reset_motor_angle()
{
  for (int i = 0; i < 20; i++)
  {
    m1.read_multi_turns_angle();
    //delay(10);
    m1.receive_CAN_data();
    m1.motorAngle_offset = m1.motorAngle_raw;
    //delay(10);
  }
}

void Wait(unsigned long delay_control)
{
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;

  do {
    Time_Control = micros() - Time_start;
  }
  while (Time_Control < Time_Delta);

}
