//IMU-based control - Teensy 3.6

#include "Motor_Control_Pediatric_V2.h"
#include <SPI.h>
#include <FlexCAN.h>
#include "WL_IMU.h"
#include <Arduino.h>
#include <stdio.h>
//#include "ads1292r.h"
//#include "CIC.h"
//#include <SD.h>

// *** for RingBuf *** //
#include "SdFat.h"
#include "RingBuf.h"

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 2000 // 500 Hz

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
// #define LOG_FILE_SIZE 10*25000*600  // Size to log 10 byte lines at 25 kHz for more than ten minutes = 150,000,000 bytes.
// #define LOG_FILE_SIZE 100 * 500 * 80 // Size to log 10 byte lines at 500 Hz for more than 80 seconds =  bytes.
#define LOG_FILE_SIZE 220 * 500 * 600 // Size to log 10 byte lines at 500 Hz for more than 80 seconds = 66,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400*512 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400 * 512 * 10 / 50 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 250 * 500 * 1 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.

// #define LOG_FILENAME "Walking.csv"
//#define LOG_FILENAME "0407_Powered_Jennifer_Walking_bd0.04_0.00_kd_1_0.6_0_0_0_0_newtau.csv"
#define LOG_FILENAME "2022-05-24-Weibo-Walking_Powered_06.csv"

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

int stopFlag = 0;
// *** End for RingBuf *** //

CAN_message_t msgR;
struct CAN_filter_t defaultMask;
uint32_t ID_offset = 0x140;

///////////////////////////////////////////////////////

char user_sex = 'M'; // M for male, F for female
float user_weight = 70; // [kg]
float user_height = 1.75; // [m]

int assist_mode = 3;

double Gain_L = 1;
double Gain_R = 1;
double Gain_common = 0.3;

int current_limitation = 5;  //(unit Amp)

///////////////////////////////////////////////////////

String mode = "start";

double Gear_ratio = 6; //The actuator gear ratio, will enfluence actuator angle and angular velocity
double Torque_const = 1.67; // Torque constant already takes into account the gear ratio

int Stop_button = 0;    // Stop function
int saveDataFlag = 1;

uint32_t Motor_ID1 = 1; //1 Motor Can Bus ID, left leg, loadcell: port 1, positive current = extension // updated on 2022-04-01 2
uint32_t Motor_ID2 = 2; //2 Motor Can Bus ID, right leg, loadcell: port 2, positive current = flexion // updated on 2022-04-01 3
int CAN_ID = 0;         // CAN port from Teensy

const double p1 = -1.027e-09;
const double p2 =  2.757e-07;
const double p3 = -3.17e-05;
const double p4 =  0.004151;
const double p5 = -0.04245;

const double a1 = 508.5;
const double b1 = 0.02148;
const double c1 = -0.6065;
const double a2 = 455;
const double b2 = 0.02248;
const double c2 = 2.479;

float ua_wc = 0;
float ua_hc = 0;
float ua_COMc = 0;

float fa_wc = 0;
float fa_hc = 0;
float fa_COMc = 0;

float h_wc = 0;
float h_hc = 0;
float h_COMc = 0;

float ua_weight = 0;
float ua_length = 0;
float ua_moment_arm = 0;

float fa_weight = 0;
float fa_length = 0;
float fa_moment_arm = 0;

float h_weight = 0;
float h_length = 0;
float h_moment_arm = 0;

float coeff_weight = 0;
float coeff_height = 0;
float coeff_COM = 0;

float angle_threshold = 20;
float ratio_L = 0;
float ratio_R = 0;
float arm_elevation_L;
float arm_elevation_R;
float moment_arm_L = 0;
float moment_arm_R = 0;
float torque_bio_L = 0;
float torque_bio_R = 0;
float motor_torque_L = 0;
float motor_torque_R = 0;


float radius_motor_pulley = 0.046; // [m]
float radius_arm = 0.233; //[m]

Motor_Control_Pediatric_V2 m1(Motor_ID1, CAN_ID, Gear_ratio); //Create motor object see Motor_Control_Pediatric_V2.h
Motor_Control_Pediatric_V2 m2(Motor_ID2, CAN_ID, Gear_ratio); //Create motor object see Motor_Control_Pediatric_V2.h
// ads1292r torque_sensor1;                                   //Create torque sensor object see ads1292r.h
IMU imu;                                                      //Create IMU object see WL_IMU.h

double Fsample = 500;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
double Fsample_ble = 100;    // [Hz] Bluetooth sending data frequency
unsigned long current_time = 0;
unsigned long previous_time = 0;                                                        // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                                    // used to control the Bluetooth communication frequency
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);               // used to control the teensy controller frequency
unsigned long Tinterval_ble_microsecond = (unsigned long)(1000000 / Fsample_ble);       // used to control the Bluetooth communication frequency

double Cur_command_L = 0;
double Cur_command_R = 0;
double torque_command_L = 0;
double torque_command_R = 0;

// Trigger
int triggerOn = 0;
int triggerPin = A9; //reading pin
int triggerVal = 0;  // analog trigger pin value

// Data logging
int isLogging = 0;
int taskIdx = 1;
int conditionIdx = 1; // baseline=1, sham=2, powered=3
int trialIdx = 1;

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy

int angle_L_ble = 0;                //left knee angle
int angle_R_ble = 0;                //right knee angle
int current_command_L_ble = 0; //current reference(A) for inner loop current control
int current_command_R_ble = 0;
int current_L_ble = 0;
int current_R_ble = 0;
int torque_command_L_ble = 0; //total torque reference(N-m)
int torque_command_R_ble = 0;
int torque_L_ble = 0;             //actual torque(N-m)(measured by torque sensor)
int torque_R_ble = 0;
int motor_angle_L_ble = 0;
int motor_angle_R_ble = 0;
int motor_speed_L_ble = 0;
int motor_speed_R_ble = 0;

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

double Left_Torque_Command;
double Right_Torque_Command;
int Left_Torque = 0;
int Right_Torque = 0;


double relTime = 0.0;
int SDcounter = 0;
int SDtotalCounter = 0;
String dataBuffer = "";
#define charBufferSize 100
char charBuffer[charBufferSize];
long charBufferPosition = 0;
//File dataFile;

//File dataFile = SD.open("walking.txt", FILE_WRITE);

void setup()
{
  // put your setup code here, to run once:
  delay(3000);
  Serial.begin(115200);  //used for communication with computer.
  Serial4.begin(115200); //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  initial_CAN();
  
  // torque_sensor1.Torque_sensor_initial();                                           //initial the torque sensor see ads1292r.cpp.
  // torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 2, 0.0003446 * (-1) * 2.35); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.
  // torque_sensor1.Torque_sensor_offset_calibration();                                //Auto offset the torque sensor to zero. see ads1292r.cpp.
  
  delay(500);

  if (user_sex == 'M')
  {
    ua_wc = 3.25/100;
    ua_hc = 17.2/100;
    ua_COMc = 43.6/100;
    
    fa_wc = 1.87/100;
    fa_hc = 15.7/100;
    fa_COMc = 43.0/100;
    
    h_wc = 0.65/100;
    h_hc = 5.75/100;
    h_COMc = 46.8/100;
  }
  else if (user_sex == 'F')
  {
    ua_wc = 2.9/100;
    ua_hc = 17.3/100;
    ua_COMc = 45.8/100;
    
    fa_wc = 1.57/100;
    fa_hc = 16.0/100;
    fa_COMc = 43.4/100;
    
    h_wc = 0.5/100;
    h_hc = 5.75/100;
    h_COMc = 46.8/100;
  }

  ua_weight = user_weight*ua_wc;
  ua_length = user_height*ua_hc;
  ua_moment_arm = ua_length*ua_COMc;

  fa_weight = user_weight*fa_wc;
  fa_length = user_height*fa_hc;
  fa_moment_arm = ua_length + fa_length*fa_COMc;
  
  h_weight = user_weight*h_wc;
  h_length = user_height*h_hc;
  h_moment_arm = ua_length + fa_length + h_length*h_COMc;
  
  imu.Gain_E = 1;         //Extension gain for delay output feedback control
  imu.Gain_F = 1;         //Flexion gain for delay output feedback control  DOFC.Gain_E = 10;            //Extension gain for delay output feedback control
  imu.STS_Gain = 0;
  imu.delaypoint = 0;     //realative to delay time (delaypoint*sampletime=delaytime) for delay output feedback control
  imu.alpha = 5;
  imu.beta = 2;
  imu.angleRelativeThreshold = 20;
  
  m1.init_motor(); // Strat the CAN bus communication & Motor Control
  delay(100);
  m2.init_motor(); // Strat the CAN bus communication & Motor Control
  delay(100);
  
  reset_motor_angle();
  
  CurrentControlSetup();

  // init trigger
  pinMode(triggerPin, INPUT);
  pinMode(A8, OUTPUT);  
}

void loop()
{
  while (stopFlag)
  {  };
  CurrentControl();
}

void CurrentControlSetup()
{
  imu.INIT(); //Initialize IMU;
  delay(500);
  imu.INIT_MEAN();
  current_time = micros();
  previous_time = current_time;
  previous_time_ble = current_time;
}

void CurrentControl()
{
  ////******IMU+Current Control Example Torque Constant 0.6 Nm/A**********////////
  imu.READ();                          //Check if IMU data available and read it. the sample rate is 100 hz
  // torque_sensor1.Torque_sensor_read(); //Check if torque sensor1 is available // Vahid
  current_time = micros();             //query current time (microsencond)

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
      Compute_Cur_Commands(); // Vahid
    }
    Cur_limitation();

    //Compute_Torque_Commands();

    //print_Data();
    //print_Current_Data();
    //print_Torque_Data();   
    //print_IMU_Data();
    
    m1.send_current_command(Cur_command_L);
    receive_CAN_data();
    m2.send_current_command(Cur_command_R);
    receive_CAN_data();
    
    m1.read_multi_turns_angle(); //read angle and angle velocity
    receive_CAN_data();
    m2.read_multi_turns_angle(); //read angle and angle velocity
    receive_CAN_data();
//
//    if ((m1.motorAngle > 180) || (m2.motorAngle > 180))
//    {
//      stopFlag = 1;
//    }
    
    // Read trigger signal
    triggerVal = analogRead(triggerPin);
    digitalWrite(A8, LOW); //pin converted into GND
    if (triggerVal < 400)
    {
      triggerOn = 0;
    }
    else
    {
      triggerOn = 1;
    }
    //Serial.print(triggerVal); Serial.print(" ");
    //Serial.print(triggerOn);

    if (isLogging)
    {
      logData3();
    }

    previous_time = current_time; //reset previous control loop time
    relTime += Tinterval_microsecond / 1000;
  }

  //********* use to control the Bluetooth communication frequency **********//
  if (current_time - previous_time_ble > Tinterval_ble_microsecond)
  {
    receive_ble_Data();
    send_ble_Data(); // send the BLE data
    previous_time_ble = current_time;
        
  }
  if (Serial.available())
  {
    char cmd = (char)Serial.read();
    if (cmd == 's')
    {
      stopFlag = 1;
      Serial.println("Stopped");
      if (saveDataFlag)
      {
        SDCardSaveToFile();
      }
    }
  }
}

void SDCardSaveToFile()
{
  rb.sync();
  file.truncate();
  file.rewind();
  file.close();
}

void Compute_Cur_Commands()
{
  if (assist_mode == 1)
  {
    mode = "Constant Signal";
    Cur_command_L = +1 * Gain_L * Gain_common;
    Cur_command_R = -1 * Gain_R * Gain_common;
  }
  else if (assist_mode == 2)
  {
    mode = "Sine Wave";
    Cur_command_L = +Gain_L/2 + Gain_L/2 * sin(2 * PI * current_time / 1000000);  // [Amp]
    Cur_command_R = -Gain_R/2 + Gain_R/2 * sin(2 * PI * current_time / 1000000);  // [Amp]
  }
  else if (assist_mode == 3)
  {
    mode = "Gravity Compensation";
    //arm_abduction_L = -imu.LTx;
    //arm_flexion_L = -imu.LTz;
    //arm_abduction_R = -imu.RTx;
    //arm_flexion_R = imu.RTz;

    arm_elevation_L = cos(imu.LTx*PI/180)*cos(imu.LTy*PI/180);
    arm_elevation_L = acos(arm_elevation_L)*180/PI;
    arm_elevation_R = cos(imu.RTx*PI/180)*cos(imu.RTy*PI/180);
    arm_elevation_R = acos(arm_elevation_R)*180/PI;

    if (arm_elevation_L > angle_threshold)
    {
      torque_bio_L = Compute_Biological_Torque(arm_elevation_L);
      moment_arm_L = Compute_moment_arm(arm_elevation_L)/1000;
      motor_torque_L = torque_bio_L*radius_motor_pulley/moment_arm_L;
      Cur_command_L = +Gain_L*Gain_common*motor_torque_L/Torque_const;
    }
    else
    {
      Cur_command_L = 0;
    }
    
    if (arm_elevation_R > angle_threshold)
    {
      torque_bio_R = Compute_Biological_Torque(arm_elevation_R);
      moment_arm_R = Compute_moment_arm(arm_elevation_R)/1000;
      motor_torque_R = torque_bio_R*radius_motor_pulley/moment_arm_R;
      Cur_command_R = -Gain_R*Gain_common*motor_torque_R/Torque_const;
    }
    else
    {
      Cur_command_R = 0;
    }
    
  }
  
  else if (assist_mode == 4)
  {
  }
  else if (assist_mode == 5) //Serial communication with High level system
  {
    mode = "High Level Control";
    send_serial_Data_Highlevel(); 
    receive_serial_Data_Highlevel();
    Cur_command_L =  Left_Torque_Command / Torque_const; //2.2;
    Cur_command_R = Right_Torque_Command / Torque_const; //2.2;
  }
  else if (assist_mode == 100)
  {
    mode = "Zero Current";
    Cur_command_L = 0;
    Cur_command_R = 0;
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
  //torque_command_L = Cur_command_L * Torque_const * Gear_ratio; //0.232 * 9;//2.2;
  //torque_command_R = Cur_command_R * Torque_const * Gear_ratio; //0.232 * 9;//2.2;
}

float Compute_Biological_Torque(float elevation_angle)
{
  float torque = (ua_weight*ua_moment_arm + fa_weight*fa_moment_arm + h_weight*h_moment_arm)*9.81*sin(elevation_angle*PI/180);
  return torque;
}

float Compute_ratio(float elevation_angle)
{
  float ratio = p1*pow(elevation_angle,4) + p2*pow(elevation_angle,3) + p3*pow(elevation_angle,2) + p4*elevation_angle + p5;
  return ratio;
}

float Compute_moment_arm(float elevation_angle)
{ 
  float moment_arm = a1*sin(b1*elevation_angle + c1) + a2*sin(b2*elevation_angle + c2);
  return moment_arm;
}

void receive_ble_Data()
{
  if (Serial4.available() >= 20)
  {
    // Serial.println("-------------New data received-------------------");
    data_rs232_rx[0] = Serial4.read();
    if (data_rs232_rx[0] == 165)
    {
      data_rs232_rx[1] = Serial4.read();
      if (data_rs232_rx[1] == 90)
      {
        data_rs232_rx[2] = Serial4.read();
        if (data_rs232_rx[2] == 20)
        {
          Serial4.readBytes(&data_rs232_rx[3], 17);
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
            user_height = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
            Serial.print("User height [m]: ");
            Serial.println(user_height);
          }
          else if (data_rs232_rx[3] == 6)
          {
            user_weight = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 10.0;
            Serial.print("User weight [kg]: ");
            Serial.println(user_weight);
          }
          else if (data_rs232_rx[3] == 7)
          {
            int sex_value = int(data_rs232_rx[4]);
            if (sex_value == 0){user_sex = 'M';}
            else if (sex_value == 1){user_sex = 'F';}
            Serial.print("User sex: ");
            Serial.println(user_sex);
          }
          else if (data_rs232_rx[3] == 10)
          {
            reset_motor_angle();
            Serial.println("The angle of motor has been reset");
            imu.INIT_MEAN();
            Serial.println("The angle of IMUs has been reset");
          }
          else if (data_rs232_rx[3] == 20)
          {
            isLogging = int(data_rs232_rx[7]);
            if (isLogging == 1)
            {
              taskIdx = int(data_rs232_rx[4]);
              conditionIdx = int(data_rs232_rx[5]);
              trialIdx = int(data_rs232_rx[6]);
              String taskName;
              if (taskIdx == 1)
              {
                taskName = "task_A";
              }
              else if (taskIdx == 2)
              {
                taskName = "task_B";
              }
              String stringOne = taskName + '-';

              String conditionName;
              if (conditionIdx == 1)
              {
                conditionName = "Baseline";
              }
              else if (conditionIdx == 2)
              {
                conditionName = "Sham";
              }
              else if (conditionIdx == 3)
              {
                conditionName = "Powered";
              }
              else
              {
                conditionName = "Others";
              }
              String stringTwo = stringOne + conditionName + "-Trial";
              String logFileName = stringTwo + String(trialIdx) + ".csv";
              SDCardSetup(logFileName.c_str());
              relTime = 0.0;
              Serial.println("Data logging started......");
            }
            else if (isLogging == 0)
            {
              SDCardSaveToFile();
              Serial.println("Data logging stopped......");
            }
          }
        }
      }
    }
  }
}

void send_ble_Data()
{
  angle_L_ble = arm_elevation_L * 100;
  angle_R_ble = arm_elevation_R * 100;
  current_command_L_ble = Cur_command_L * 100; // Gui flexion is negative
  current_command_R_ble = Cur_command_R * 100;  // Gui flexion is negative  
  current_L_ble = m1.iq_A * 100;
  current_R_ble = m2.iq_A * 100;
  torque_command_L_ble = torque_command_L * 100; // Gui flexion is negative
  torque_command_R_ble = torque_command_R * 100; // Gui flexion is negative
  torque_L_ble = m1.iq_A * Torque_const * 100; // +m1.iq_A * 0.232 * 9 * 100; //torque_sensor1.torque[0]   // motor torque constant = 0.232 Nm/A, gear ratio = 9;
  torque_R_ble = m2.iq_A * Torque_const * 100; // -m2.iq_A * 0.232 * 9 * 100; //torque_sensor1.torque[1]   // motor torque constant = 0.232 Nm/A, gear ratio = 9;
  motor_angle_L_ble = m1.motorAngle * 100; // [deg]
  motor_angle_R_ble = m2.motorAngle * 100; // [deg]
  motor_speed_L_ble = m1.speed_value / 180 * 3.1415 * 100; // [rad/s]
  motor_speed_R_ble = m2.speed_value / 180 * 3.1415 * 100; // [rad/s]

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0] = 165;
  data_ble[1] = 90;
  data_ble[2] = datalength_ble;
  
  data_ble[3] = angle_L_ble;
  data_ble[4] = angle_L_ble >> 8;
  data_ble[5] = angle_R_ble;
  data_ble[6] = angle_R_ble >> 8;
  data_ble[7] = current_command_L_ble;
  data_ble[8] = current_command_L_ble >> 8;
  data_ble[9] = current_command_R_ble;
  data_ble[10] = current_command_R_ble >> 8;
  data_ble[11] = current_L_ble;
  data_ble[12] = current_L_ble >> 8;
  data_ble[13] = current_R_ble;
  data_ble[14] = current_R_ble >> 8;
  data_ble[15] = torque_command_L_ble;
  data_ble[16] = torque_command_L_ble >> 8;
  data_ble[17] = torque_command_R_ble;
  data_ble[18] = torque_command_R_ble >> 8;
  data_ble[19] = torque_L_ble;
  data_ble[20] = torque_L_ble >> 8;
  data_ble[21] = torque_R_ble;
  data_ble[22] = torque_R_ble >> 8;
  data_ble[23] = motor_angle_L_ble;
  data_ble[24] = motor_angle_L_ble >> 8;
  data_ble[25] = motor_angle_R_ble;
  data_ble[26] = motor_angle_R_ble >> 8;
  data_ble[27] = motor_speed_L_ble;
  data_ble[28] = motor_speed_L_ble >> 8;
  data_ble[29] = motor_speed_R_ble;
  data_ble[30] = motor_speed_R_ble >> 8;

  Serial4.write(data_ble, datalength_ble);
}

//******************Receive high level controller Command****************//
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
        Serial.readBytes(&data_highlevel_rx[2], 5); //int data_rs232_rx[7]
        //Highlevel_Data_Length_Receive = int(data_highlevel_rx[2]); // read data length
        Left_Torque_Command = ((int16_t)(((int16_t)data_highlevel_rx[3]) | ((int16_t)data_highlevel_rx[4] << 8)));
        Right_Torque_Command = ((int16_t)(((int16_t)data_highlevel_rx[5]) | ((int16_t)data_highlevel_rx[6] << 8)));
        Left_Torque_Command = Left_Torque_Command / 100;
        Right_Torque_Command = Right_Torque_Command / 100;
      }
    }
  }
}

//******************Send high level controller Command****************//
void send_serial_Data_Highlevel()
{
  LK_highlevel = imu.LKx * 100;
  RK_highlevel = imu.RKx * 100;
  TK_highlevel = imu.TKx * 100;
  LT_highlevel = imu.LTx * 100;
  RT_highlevel = imu.RTx * 100;
  LS_highlevel = imu.LSx * 100;
  RS_highlevel = imu.RSx * 100;
  Left_Torque  = +m1.iq_A * Torque_const * 100; //torque_sensor1.torque[0] * 100;
  Right_Torque = -m2.iq_A * Torque_const * 100; //torque_sensor1.torque[1] * 100;

  data_serial_highlevel[0] = 165;
  data_serial_highlevel[1] = 90;
  data_serial_highlevel[2] = Highlevel_Data_Length_Send;
  
  data_serial_highlevel[3] = Left_Torque >> 8;
  data_serial_highlevel[4] = Left_Torque;
  data_serial_highlevel[5] = Right_Torque >> 8;
  data_serial_highlevel[6] = Right_Torque;
  data_serial_highlevel[7] = LK_highlevel >> 8;
  data_serial_highlevel[8] = LK_highlevel;
  data_serial_highlevel[9] = RK_highlevel >> 8;
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
    if (msgR.id == (ID_offset + Motor_ID1))
    {
      m1.DataExplanation(msgR);
    }
    else if (msgR.id == (ID_offset + Motor_ID2))
    {
      m2.DataExplanation(msgR);
    }
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

void SDCardSetup()
{
  sd.remove(LOG_FILENAME);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC))
  {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE))
  {
    Serial.println("preAllocate failed\n");
    file.close();
    return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.println("Data logging initialized.");
}

void SDCardSetup(const char* logFileName)
{
  sd.remove(logFileName);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(logFileName, O_RDWR | O_CREAT | O_TRUNC))
  {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE))
  {
    Serial.println("preAllocate failed\n");
    file.close();
    return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.print("Data logging initialized. File name = ");
  Serial.println(logFileName);
}

//*** Ringbuf ***//
void logData3()
{
  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;

  // Min spare micros in loop.
  //int32_t minSpareMicros = INT32_MAX;

  // Start time.
  //uint32_t logTime = micros();
  // Log data until Serial input or file full.
  //  while (!Serial.available()) {
  // Amount of data in ringBuf.
  size_t n = rb.bytesUsed();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20))
  {
    Serial.println("File full - quitting.");
    return; // break;
  }
  if (n > maxUsed)
  {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy())
  {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512))
    {
      Serial.println("writeOut failed");
      return; //break;
    }
  }
  // Time for next point.
  //  logTime += LOG_INTERVAL_USEC;
  //  int32_t spareMicros = logTime - micros();
  //  if (spareMicros < minSpareMicros) {
  //    minSpareMicros = spareMicros;
  //  }
  //  if (spareMicros <= 0) {
  //    Serial.print("Rate too fast ");
  //    Serial.println(spareMicros);
  //    break;
  //  }
  // Wait until time to log data.
  //  while (micros() < logTime) {}

  // Read ADC0 - about 17 usec on Teensy 4, Teensy 3.6 is faster.
  //  uint16_t adc = analogRead(0);
  // Print spareMicros into the RingBuf as test data.
     rb.print(relTime); rb.write(" ");
     rb.print(imu.TKx); rb.write(" ");
     rb.print(imu.TKy); rb.write(" ");
     rb.print(imu.TKz); rb.write(" ");
     rb.print(imu.LTx); rb.write(" ");
     rb.print(imu.LTy); rb.write(" ");
     rb.print(imu.LTz); rb.write(" ");
     rb.print(imu.RTx); rb.write(" ");
     rb.print(imu.RTy); rb.write(" ");
     rb.print(imu.RTz); rb.write(" ");
     rb.print(imu.LSx); rb.write(" ");
     rb.print(imu.LSy); rb.write(" ");
     rb.print(imu.LSz); rb.write(" ");
     rb.print(imu.RSx); rb.write(" ");
     rb.print(imu.RSy); rb.write(" ");
     rb.print(imu.RSz); rb.write(" ");
     rb.print(imu.TKAVx); rb.write(" ");
     rb.print(imu.TKAVy); rb.write(" ");
     rb.print(imu.TKAVz); rb.write(" ");
     rb.print(imu.LTAVx); rb.write(" ");
     rb.print(imu.LTAVy); rb.write(" ");
     rb.print(imu.LTAVz); rb.write(" ");
     rb.print(imu.RTAVx); rb.write(" ");
     rb.print(imu.RTAVy); rb.write(" ");
     rb.print(imu.RTAVz); rb.write(" ");
     rb.print(imu.LSAVx); rb.write(" ");
     rb.print(imu.LSAVy); rb.write(" ");
     rb.print(imu.LSAVz); rb.write(" ");
     rb.print(imu.RSAVx); rb.write(" ");
     rb.print(imu.RSAVy); rb.write(" ");
     rb.print(imu.RSAVz); rb.write(" ");
     rb.print(Cur_command_L); rb.write(" ");
     rb.print(Cur_command_R); rb.write(" ");
     rb.print(m1.iq_A); rb.write(" ");
     rb.print(m2.iq_A); rb.write(" ");
     rb.print(m1.motorAngle); rb.write(" ");
     rb.print(m2.motorAngle); rb.write(" ");
     rb.print(m1.speed_value); rb.write(" ");
     rb.print(m2.speed_value); rb.write(" "); 
     rb.println();
     //rb.print(triggerOn); //rb.write(" ");
     //rb.print(triggerVal); //rb.write(" ");
     //rb.print("\n");
  // Print adc into RingBuf.
  //  rb.println(adc);
  if (rb.getWriteError())
  {
    // Error caused by too few free bytes in RingBuf.
    Serial.println("WriteError");
    return; //break;
  }
  
}

//**************Plot Data*****************//

void print_Data()
{
  //Serial.print(current_time/1e6); Serial.print("   ");
  Serial.print(arm_elevation_L); Serial.print("  ");
  Serial.print(arm_elevation_R); Serial.print("  ");
  Serial.print(torque_bio_L); Serial.print("  ");
  Serial.print(torque_bio_R); Serial.print("  ");
  //Serial.print(ratio_L); Serial.print("  ");
  //Serial.print(ratio_R); Serial.print("  ");
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

void print_Current_Data()
{
    Serial.print(Cur_command_L); Serial.print("   ");
    Serial.print(Cur_command_R); Serial.print("   ");
//    Serial.println();
}

void print_Torque_Data()
{
    Serial.print(torque_command_L); Serial.print("   ");
    Serial.print(torque_command_R); Serial.print("   ");
//    Serial.println();
}

void print_IMU_Data()
{
//     Serial.print(imu.TKx); Serial.print("   ");
//     Serial.print(imu.TKy); Serial.print("   ");
//     Serial.print(imu.TKz); Serial.print("   ");
//     Serial.print(imu.LTx); Serial.print("   ");
//     Serial.print(imu.LTy); Serial.print("   ");
//     Serial.print(imu.LTz); Serial.print("   ");
//     Serial.print(imu.RTx); Serial.print("   ");
//     Serial.print(imu.RTy); Serial.print("   ");
//     Serial.print(imu.RTz); Serial.print("   ");
     Serial.print(imu.LSx); Serial.print("   ");
     Serial.print(imu.LSy); Serial.print("   ");
     Serial.print(imu.LSz); Serial.print("   ");
     Serial.print(imu.RSx); Serial.print("   ");
     Serial.print(imu.RSy); Serial.print("   ");
     Serial.print(imu.RSz); Serial.print("   ");
//     Serial.print(imu.TKAVx); Serial.print("   ");
//     Serial.print(imu.TKAVy); Serial.print("   ");
//     Serial.print(imu.TKAVz); Serial.print("   ");
//     Serial.print(imu.LTAVx); Serial.print("   ");
//     Serial.print(imu.LTAVy); Serial.print("   ");
//     Serial.print(imu.LTAVz); Serial.print("   ");
//     Serial.print(imu.RTAVx); Serial.print("   ");
//     Serial.print(imu.RTAVy); Serial.print("   ");
//     Serial.print(imu.RTAVz); Serial.print("   ");
     Serial.print(imu.LSAVx); Serial.print("   ");
     Serial.print(imu.LSAVy); Serial.print("   ");
     Serial.print(imu.LSAVz); Serial.print("   ");
     Serial.print(imu.RSAVx); Serial.print("   ");
     Serial.print(imu.RSAVy); Serial.print("   ");
     Serial.print(imu.RSAVz); Serial.print("   ");
//     Serial.println("   ");
}

void plot_debug_data()
{
}
