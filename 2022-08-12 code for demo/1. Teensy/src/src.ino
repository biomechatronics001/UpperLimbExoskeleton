//11 Aug 2022

#include "Motor_Control_Pediatric_V2.h"
#include <SPI.h>
#include <FlexCAN.h>
#include "ads1292r.h"
#include "WL_IMU.h"
#include <Arduino.h>

int assist_mode = 1;
double Gear_ratio = 6; //The actuator gear ratio, will enfluence actuator angle and angular velocity
double Torque_const = 1.67;
double Gain_L = 1;
double Gain_R = 1;
double Gain_common = 0;
int current_limitation = 6;  //(unit Amp)

int Stop_button = 0;    // Stop function
String mode = "start";

CAN_message_t msgR;
struct CAN_filter_t defaultMask;
uint32_t ID_offset=0x140;
uint32_t Motor_ID1 = 1;      // Motor Can Bus ID, left leg, loadcell: port 1
uint32_t Motor_ID2 = 2;      // Motor Can Bus ID, right leg, loadcell: port 2
int CAN_ID = 0;         // CAN port from Teensy

Motor_Control_Pediatric_V2 m1(Motor_ID1, CAN_ID, Gear_ratio);  //Create motor object see Motor_Control_Pediatric_V2.h
Motor_Control_Pediatric_V2 m2(Motor_ID2, CAN_ID, Gear_ratio);  //Create motor object see Motor_Control_Pediatric_V2.h
ads1292r torque_sensor1;                //Create torque sensor object see ads1292r.h
IMU imu;                                //Create IMU object see WL_IMU.h

double Fsample = 500;                   // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
//double Tsample = 1 / Fsample;           // sample period (second)
double Fsample_ble = 100;               // [Hz] Bluetooth sending data frequency
unsigned long current_time = 0;
unsigned long previous_time = 0;        // used to control the controller sample rate.
unsigned long previous_time_ble = 0;    // used to control the Bluetooth communication frequency
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);             // used to control the teensy controller frequency
unsigned long Tinterval_ble_microsecond = (unsigned long)(1000000 / Fsample_ble);     // used to control the Bluetooth communication frequency

double Cur_command_L = 0;
double Cur_command_R = 0;

//***Data sent via bluetooth
char datalength_ble = 32;          // Bluetooth Data Length
char data_ble[60] = {0};           // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};      // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy
int LK_ble = 0;                    //left knee angel
int RK_ble = 0;                    //right knee angle
int current_command_L_ble = 0;     //current reference(A) for inner loop current control
int current_command_R_ble = 0;
int torque_command_L_ble = 0;      //torque reference (Nm)
int torque_command_R_ble = 0;
int torque_L_ble = 0;              //actual torque (Nm) measured by torque sensor
int torque_R_ble = 0;
int angle_L_ble = 0;
int angle_R_ble = 0;

double torque_command_L = 0;
double torque_command_R = 0;

//***********High Level Communication*********//
char data_serial_highlevel[101] = {0};
char data_highlevel_rx[7] = {0}; //data high level communication
char Highlevel_Data_Length_Send = 101;

int LK_highlevel = 0;
int RK_highlevel = 0;
int TK_highlevel = 0;
int LT_highlevel = 0;
int RT_highlevel = 0;
int LS_highlevel = 0;
int RS_highlevel = 0;

int Left_Torque = 0;
int Right_Torque = 0;
double Left_Torque_Command;
double Right_Torque_Command;

void setup() 
{
  // put your setup code here, to run once:
  delay(3000);
  Serial.begin(115200);       //used for communication with computer.
  Serial4.begin(115200);      //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  initial_CAN();
  torque_sensor1.Torque_sensor_initial();     //initial the torque sensor see ads1292r.cpp.
  torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 2, 0.0003446 * (-1) * 2.35); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.
  torque_sensor1.Torque_sensor_offset_calibration();      //Auto offset the torque sensor to zero. see ads1292r.cpp.
  delay(500);
  //imu.Gain_E = 0;         //Extension gain for delay output feedback control
  //imu.Gain_F = 0;         //Flexion gain for delay output feedback control
  //imu.delaypoint = 10;    //realative to delay time (delaypoint*sampletime=delaytime) for delay output feedback control
  m1.init_motor();        // Strat the CAN bus communication & Motor Control
  m2.init_motor();        // Strat the CAN bus communication & Motor Control
  CurrentControlSetup();  //initialize current control and IMU control. this is use for direct current control without torque PID loop (torque reference/torque constant= current reference -> current PID loop)
  //PositionControlSetup();
}

void loop()
{
  //receive_CAN_data();
  CurrentControl();
  //PositionControl();
}

void CurrentControlSetup()
{
  imu.INIT();    //Initialize IMU;
  delay(500);
  imu.INIT_MEAN();
  current_time = micros();
  previous_time = current_time;
  previous_time_ble = current_time;
}

void CurrentControl()
{
  ////******Torque Constant 0.6 Nm/A**********////////
  imu.READ();   //Check if IMU data available and read it. The sample rate is 100 Hz
  //print_imu_data();
  
  torque_sensor1.Torque_sensor_read(); //Check if torque sensor1 is available
  //print_torque_sensor_data();

  current_time = micros();    //query current time (microsencond)

  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    if (Stop_button) //stop
    {
      Cur_command_L = 0;
      Cur_command_R = 0;
    }
    else
    {
      Compute_Cur_Commands();
      Cur_limitation();
      Compute_Torque_Commands();
    }

    m1.send_current_command(Cur_command_L);
    receive_CAN_data();
    m2.send_current_command(Cur_command_R);
    receive_CAN_data();
    
    m1.read_multi_turns_angle();//read angle and angular velocity
    receive_CAN_data();
    m2.read_multi_turns_angle();//read angle and angular velocity
    receive_CAN_data();

    if (m1.motorAngle > 300 || m2.motorAngle > 300)
    {
      assist_mode = 5;
    }

    previous_time = current_time; //reset previous control loop time
  }

  //********* use to control the Bluetooth communication frequency **********//
  if (current_time - previous_time_ble > Tinterval_ble_microsecond)
  {
    receive_ble_Data();
    send_ble_Data();
    
    print_Data();
    
    previous_time_ble = current_time;
  }
  
}

void Compute_Cur_Commands()
{
  if (assist_mode == 1) // Constant Signal
  {
    mode = "Constant Signal";
    Cur_command_L = +1 * Gain_L * Gain_common;
    Cur_command_R = -1 * Gain_R * Gain_common;
  }
  else if (assist_mode == 2) // Sinusoidal Signal
  {
    mode = "Sine Wave";
    Cur_command_L = +Gain_L/2 + Gain_L/2 * sin(2 * PI * current_time / 1000000);  //(unit Amp);
    Cur_command_R = -Gain_R/2 + Gain_R/2 * sin(2 * PI * current_time / 1000000);  //(unit Amp);
  }
  else if (assist_mode == 3) // Serial communication with High level system
  {
    mode = "High Level Control";
    send_serial_Data_Highlevel(); 
    receive_serial_Data_Highlevel();
    Cur_command_L = Left_Torque_Command / Torque_const;
    Cur_command_R = Right_Torque_Command / Torque_const;
  }
  else if (assist_mode == 4) // IMU-based control
  {
    mode = "IMU-Based";
    Cur_command_L =  imu.DOTC[0] / Torque_const; //this is for bilateral walking assistance_left leg
    Cur_command_R =  -imu.DOTC[1] / Torque_const; //this is for bilateral walking assistance_right leg
  }
  else if (assist_mode == 5) // STOP
  {
    mode = "STOP";
    Cur_command_L =  0;
    Cur_command_R =  0;
  }
}

void Cur_limitation()
{
  //************* Current limitation *************//
  Cur_command_L = min(+current_limitation, Cur_command_L);
  Cur_command_L = max(-current_limitation, Cur_command_L);
  Cur_command_R = min(+current_limitation, Cur_command_R);
  Cur_command_R = max(-current_limitation, Cur_command_R);
}

void Compute_Torque_Commands()
{
  torque_command_L = Cur_command_L * Torque_const;
  torque_command_R = Cur_command_R * Torque_const;
}

void receive_ble_Data()
{
  if (Serial4.available() >= 20)
  {
    Serial.println("-------------New data received-------------------");
    data_rs232_rx[0] = Serial4.read();
    if (data_rs232_rx[0] == 165)
    {
      data_rs232_rx[1] = Serial4.read();
      if (data_rs232_rx[1] == 90)
      {
        Serial4.readBytes(&data_rs232_rx[2], 18);
        if (data_rs232_rx[3] == 0)
        {
          Stop_button = int(data_rs232_rx[4]);
          if (Stop_button)
          {
            Serial.println("STOP button pressed");
          }
          else
          {
            Serial.println("START button pressed");
          }
        }
        else if (data_rs232_rx[3] == 1)
        {
          assist_mode = int(data_rs232_rx[4]);
          Serial.print("Mode: "); 
          Serial.println(assist_mode);
        }
        else if (data_rs232_rx[3] == 2)
        {
          Gain_L = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          Serial.print("Left gain: ");
          Serial.println(Gain_L);
        }
        else if (data_rs232_rx[3] == 3)
        {
          Gain_R = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          Serial.print("Right gain: ");
          Serial.println(Gain_R);
        }
        else if (data_rs232_rx[3] == 4)
        {
          Gain_common = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          Serial.print("Common gain: ");
          Serial.println(Gain_common);
        }
        else if (data_rs232_rx[3] == 5)
        {
        }
        else if (data_rs232_rx[3] == 6)
        {
        }
        else if (data_rs232_rx[3] == 7)
        {
          Serial. println("The angle of motor has been reset");
          reset_motor_angle();
          imu.INIT_MEAN();
        }
      }
    }
  }
}

void send_ble_Data()
{
  LK_ble = imu.LKx * 100;
  RK_ble = imu.RKx * 100;
  current_command_L_ble = Cur_command_L * 100;
  current_command_R_ble = Cur_command_R * 100;
  torque_command_L_ble = torque_command_L * 100;
  torque_command_R_ble = torque_command_R * 100;
  torque_L_ble = torque_sensor1.torque[0] * 100;
  torque_R_ble = torque_sensor1.torque[1] * 100;
  angle_L_ble = m1.motorAngle * 100;
  angle_R_ble = m2.motorAngle * 100;

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0] = 165;
  data_ble[1] = 90;
  data_ble[2] = datalength_ble;
  
  data_ble[3] = LK_ble;
  data_ble[4] = LK_ble >> 8;
  data_ble[5] = RK_ble;
  data_ble[6] = RK_ble >> 8;
  data_ble[7] = current_command_L_ble;
  data_ble[8] = current_command_L_ble >> 8;
  data_ble[9] =  current_command_R_ble;
  data_ble[10] = current_command_R_ble >> 8;
  data_ble[11] = torque_command_L_ble;
  data_ble[12] = torque_command_L_ble >> 8;
  data_ble[13] = torque_command_R_ble;
  data_ble[14] = torque_command_R_ble >> 8;
  data_ble[15] = torque_L_ble;
  data_ble[16] = torque_L_ble >> 8;
  data_ble[17] = torque_R_ble;
  data_ble[18] = torque_R_ble >> 8;
  data_ble[19] = angle_L_ble;
  data_ble[20] = angle_L_ble >> 8;
  data_ble[21] = angle_R_ble;
  data_ble[22] = angle_R_ble >> 8;
  
//  data_ble[19] = triggerOn;
//  data_ble[20] = triggerVal;
//  data_ble[21] = current_actual_L_ble;
//  data_ble[22] = current_actual_L_ble >> 8;
//  data_ble[23] = current_actual_R_ble;
//  data_ble[24] = current_actual_R_ble >> 8;
//  data_ble[25] = 0;
//  data_ble[26] = 0 >> 8;
//  data_ble[27] = 0;
//  data_ble[28] = 0 >> 8;
//  data_ble[29] = 0;
//  data_ble[30] = 0 >> 8;
//  data_ble[31] = 0;

  Serial4.write(data_ble, datalength_ble);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

//******************Receive high level controller command****************//
void receive_serial_Data_Highlevel()
{
  if (Serial.available() >= 7)
  {
    //Serial.println("receive");
    data_highlevel_rx[0] = Serial.read();
    if (data_highlevel_rx[0] == 165)
    {
      data_highlevel_rx[1] = Serial.read();
      if (data_highlevel_rx[1] == 90)
      {
        Serial.readBytes(&data_highlevel_rx[2], 5);//int data_rs232_rx[7]
        //Highlevel_Data_Length_Receive = int(data_highlevel_rx[2]); // read data length
        Left_Torque_Command = ((int16_t)(((int16_t)data_highlevel_rx[3]) | ((int16_t)data_highlevel_rx[4] << 8)));
        Right_Torque_Command = ((int16_t)(((int16_t)data_highlevel_rx[5]) | ((int16_t)data_highlevel_rx[6] << 8)));
        Left_Torque_Command = Left_Torque_Command / 100;
        Right_Torque_Command = Right_Torque_Command / 100;
      }
    }
  }
}

//******************Send high level controller commend****************//
void send_serial_Data_Highlevel()
{
  LK_highlevel = imu.LKx * 100;
  RK_highlevel = imu.RKx * 100;
  TK_highlevel = imu.TKx * 100;
  LT_highlevel = imu.LTx * 100;
  RT_highlevel = imu.RTx * 100;
  LS_highlevel = imu.LSx * 100;
  RS_highlevel = imu.RSx * 100;
  Left_Torque = torque_sensor1.torque[0] * 100;
  Right_Torque = torque_sensor1.torque[1] * 100;

  data_serial_highlevel[0] = 165;
  data_serial_highlevel[1] = 90;
  data_serial_highlevel[2] = Highlevel_Data_Length_Send;
  
  data_serial_highlevel[3] = Left_Torque >> 8;
  data_serial_highlevel[4] = Left_Torque;
  data_serial_highlevel[5] = Right_Torque >> 8;
  data_serial_highlevel[6] = Right_Torque;
  data_serial_highlevel[7] = LK_highlevel >> 8;
  data_serial_highlevel[8] = LK_highlevel;
  data_serial_highlevel[9] =  RK_highlevel >> 8;
  data_serial_highlevel[10] = RK_highlevel;
  data_serial_highlevel[11] = LT_highlevel >> 8;
  data_serial_highlevel[12] = LT_highlevel;
  data_serial_highlevel[29] = RT_highlevel >> 8;
  data_serial_highlevel[30] = RT_highlevel;
  data_serial_highlevel[47] = LS_highlevel >> 8;
  data_serial_highlevel[48] = LS_highlevel;
  data_serial_highlevel[65] = RS_highlevel >> 8;
  data_serial_highlevel[66] = RS_highlevel;
  data_serial_highlevel[83] = TK_highlevel >> 8;
  data_serial_highlevel[84] = TK_highlevel;

  //  Serial.print(LT_highlevel); Serial.print("  ");
  //  Serial.print(RT_highlevel); Serial.print("  ");
  //  Serial.print(LS_highlevel); Serial.print("  ");
  //  Serial.print(RS_highlevel); Serial.print("  ");
  //  Serial.print(TK_highlevel); Serial.print("  ");
  //  Serial.println();

  Serial.write(data_serial_highlevel, Highlevel_Data_Length_Send);
}

void initial_CAN()
{  
  //initial CAN Bus
  Can0.begin(1000000, defaultMask, 1, 1);
  delay(3000);
  pinMode(28, OUTPUT);
  digitalWrite(28, LOW);
  Serial.println("Can bus setup done...");
}

void receive_CAN_data()
{
  while (Can0.available() > 0)
  {
    Can0.read(msgR);  
    if(msgR.id == (ID_offset+Motor_ID1))
    {m1.DataExplanation(msgR);}
    else if(msgR.id == (ID_offset+Motor_ID2))
    {m2.DataExplanation(msgR);}   
  }
}

void reset_motor_angle()
{
  for (int i = 0; i < 20; i++)
  {
    m1.read_multi_turns_angle();
    delay(10);
    receive_CAN_data();
    m1.motorAngle_offset = m1.motorAngle_raw;
    m2.read_multi_turns_angle();
    delay(10);
    receive_CAN_data();
    m2.motorAngle_offset = m2.motorAngle_raw;
  }
}

//void print_torque_sensor_data()
//{
//*** torque_sensor1.torque[0] is the first channel ADC value, [1] is the second ADC value. it can be righ leg or left leg torque sensor, it depends on the connection.
//*** Here we conncet torque_sensor1.torque[0] to right leg. (the definition is different form the IMU and control (1: righ leg 0:left leg))
//    Serial.print(torque_sensor1.torque[0]); Serial.print("  "); // plot actual torque from ADC channel 0 
//    Serial.print(torque_sensor1.torque[1]); Serial.print("  "); // plot actual torque from ADC channel 1
//    Serial.println();
//}

//void print_imu_data()
//{
//    Serial.print(imu.LTx); Serial.print("  "); //plot left thigh angle (deg)
//    Serial.print(imu.RTx); Serial.print("  "); //plot righ thigh angle (deg)
//    Serial.print(imu.LSx); Serial.print("  "); //plot left shank angle (deg)
//    Serial.print(imu.RSx); Serial.print("  "); //plot left shank angle (deg)
//    Serial.print(imu.LKx); Serial.print("  "); //plot left knee angle (deg)
//    Serial.print(imu.RKx); Serial.print("  "); //plot right knee angle (deg)
//    Serila.println();
//}

void print_Data()
{
  Serial.print(Cur_command_L); Serial.print("  ");
  Serial.print(Cur_command_R); Serial.print("  ");
  Serial.print(m1.motorAngle); Serial.print("  "); // angle unit: degree 
  Serial.print(m2.motorAngle); Serial.print("  "); // angle unit: degree
//  Serial.print(imu.LTx); Serial.print(" ");
//  Serial.print(imu.RTx); Serial.print(" ");
//  Serial.print(m1.speed_value); Serial.print("  "); // angle velocity unit: degree per second
//  Serial.print(m2.speed_value); Serial.print("  "); // angle velocity unit: degree per second
  Serial.println();
}
