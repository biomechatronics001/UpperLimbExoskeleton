//////////////////// LIBRARIES ////////////////////

/*General Libraries*/ 
#include "Serial_Com.h"
#include <Arduino.h>
#include "MovingAverage.h"
#include <math.h>  
#include <iomanip> 
#include <cstring>  

/*Libraries for motor control*/ 
#include <FlexCAN_T4.h>   
#include "Omni_Motor_Control.h"

/*Library for IMU*/ 
#include "WL_IMU.h"

/*Library for force/torque sensor*/ 
// #include "ads1292r.h"

///////////////////////////////////////////////////////

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;   
 
/*Filter for external sensors*/
MovingAverage LTAVx(12);    
MovingAverage RTAVx(12);     
float f_LTAVx = 0;  
float f_RTAVx = 0;  

CAN_message_t msgR;   
int CAN_ID = 3;  

//////////////////// CONTROL PARAMETERS ////////////////////
// This first block includes parameters that can be changed from the GUI app //

int assist_mode = 2;    // see function Compute_Torque_Commands(): 1 constant signal, 2 sine wave, 3 gravity compensation
int Stop_button = 0;    // Stop function that can be activated from the GUI app

char user_sex = 'M'; // M for male, F for female
float user_weight = 70; // [kg]
float user_height = 1.75; // [m]

double Gain_L = 1;
double Gain_R = 1;
double Gain_common = 1;

///////////////////////////////////////////////////////

String mode = "start";

uint32_t omni_motor_ID_1 = 0; 
uint32_t omni_motor_ID_2 = 1;    

double M1_torque_command = 0;  
double M2_torque_command = 0;    

double MAX_torque_command = 12;   
double MIN_torque_command = -12;    

float initial_pos_1 = 0;       
float initial_pos_2 = 0;    

/*Fitting parameters for the relationship between moment arm and elevation angle ( see function Compute_moment_arm(float elevation_angle))*/
const double a1 = 508.5;
const double b1 = 0.02148;
const double c1 = -0.6065;
const double a2 = 455;
const double b2 = 0.02248;
const double c2 = 2.479;

float radius_motor_pulley = 0.046; // [m]

/*Anthropometric parameters to estimate the gravitational torque as a function of user weight and height*/
// Their values are set in the function setup()
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

///////////////////////////////////////////////////////

/*Create motor objects (see omni_motor_Control.h)*/
Motor_Control_Tmotor omni_m1(omni_motor_ID_1, CAN_ID);
Motor_Control_Tmotor omni_m2(omni_motor_ID_2, CAN_ID);

/*Create IMU object (see WL_IMU.h)*/
IMU imu;     

/*Create force/torque sensor object  (see ads1292.h)*/ 
// ads1292r torque_sensor1;

/*Serial Class Setup*/  
Serial_Com Serial_Com;   

/*Serial Send*/  
size_t Send_Length = 11; 
char Send[11] = { 0x31, 0x32, 0x32, 0x33, 0x33,
                  0x30, 0x31, 0x32, 0x33, 0x33,
                  0x33 };   

/*Time control, for managing the Controller and Bluetooth rate*/
double t;   
double next_t;    
double delta_t;    
double cyclespersec_ctrl = 100;    // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ble  = 20;     // [Hz] Bluetooth sending data frequency 
unsigned long t_0 = 0;             // initial time [microsecond]
unsigned long current_time = 0;
unsigned long previous_time = 0;                                           // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                       // used to control the Bluetooth communication frequency
unsigned long Tinterval_ctrl_micros = (unsigned long)(1000000 / cyclespersec_ctrl); // used to control the teensy controller frequency
unsigned long Tinterval_ble_micros  = (unsigned long)(1000000 / cyclespersec_ble);  // used to control the Bluetooth communication frequency

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy
int t_teensy = 0;  
int L_motor_torque  = 0;   
int R_motor_torque  = 0;   
int L_motor_torque_command = 0;
int R_motor_torque_command = 0;
double Rescaling_gain    = 0;
double Flex_Assist_gain  = 0;
double Ext_Assist_gain   = 0;
double Assist_delay_gain = 0;  

//***Other motor and model parameters
float tau_t_1 = 0.0;  
float tau_t_2 = 0.0;    
float pos_1 = 0.0;
float pos_2 = 0.0;
  
int l_ctl_dir = 1;  // direction left motor    
int r_ctl_dir = -1;   // direction right motor

float angle_threshold = 20; // the assistive torque is set to zero when elevation angle is less than this angle threshold
float arm_elevation_L;
float arm_elevation_R;
float moment_arm_L = 0;
float moment_arm_R = 0;
float torque_bio_L = 0;
float torque_bio_R = 0;
float motor_torque_L = 0;
float motor_torque_R = 0;

////////////////////////////////////////

// The setup() function is is executed once at the start, used for initialization
void setup() {
  delay(3000);

  /*Initialize anthropometricparameters depending on user sex*/
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

  /*Initialize Serial communications*/
  Serial.begin(115200);     //115200/9600=12
  Serial2.begin(115200);    //115200/9600=12
  Serial5.begin(115200);    //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  //Serial7.begin(115200);  // Communication with Raspberry PI or PC for High-lever controllers like RL
  Serial_Com.INIT();    
 
  /*Initialize motors and CAN communication*/
  initial_CAN();    
  initial_omni_motor();
    
  imu.INIT(); //Initialize IMU;
  delay(500);
  imu.INIT_MEAN();
  
  delay(100);
  t_0 = micros();
}  

// The loop() function runs repeatedly, containing the main program logic that executes continuously. 
void loop() {
    /*read Serial and IMU*/
    Serial_Com.READ2();  
    imu.READ();     

    current_time = micros() - t_0;  // elapsed time in microsecond         
    t = current_time/1000000.0;     // elapsed time in second 
    
    if (current_time - previous_time > Tinterval_ctrl_micros) // check control rate
    {
      if (current_time - previous_time_ble > Tinterval_ble_micros) // check bluetooth rate
      {
        Receive_ble_Data();  
        Transmit_ble_Data();
              
        print_data(); 
         
        previous_time_ble = current_time;   
      }  

      if (Stop_button) //stop
        {
          M1_torque_command = 0;
          M2_torque_command = 0;
        }
      else
        {
          Compute_Torque_Commands();
        }
      
      // clip the torque command  
      M1_torque_command = clip_torque(M1_torque_command);         
      M2_torque_command = clip_torque(M2_torque_command);           
    
      M1_torque_command = M1_torque_command * l_ctl_dir; // left side  
      M2_torque_command = M2_torque_command * r_ctl_dir; // right side
      
      //for (int i=0; i < 4; i++)   
      //{
      //omni_m1.request_pos_vel();
      //receive_torque_ctl_feedback();  //read motor CAN info

      omni_m1.request_torque();
      //receive_torque_ctl_feedback();  //read motor CAN info

      //omni_m2.request_pos_vel();
      //receive_torque_ctl_feedback();  //read motor CAN info

      omni_m2.request_torque();
      receive_torque_ctl_feedback();  //read motor CAN info
      //} 

      //send torque commands to the motors
      omni_m1.omni_torque_cmd(M1_torque_command);      
      omni_m2.omni_torque_cmd(M2_torque_command);

      //print_data_motor();      
      previous_time = current_time;   
    }  
}  

void Compute_Torque_Commands()
{
  if (assist_mode == 1)
  {
    mode = "Constant Signal";
    M1_torque_command = Gain_L * Gain_common * 0.5;
    M2_torque_command = Gain_R * Gain_common * 0.5;
  }
  else if (assist_mode == 2)
  {
    mode = "Sine Wave";
    M1_torque_command =  Gain_L * sin(2 * PI * t) * 0.7;
    M2_torque_command =  Gain_R * sin(2 * PI * t) * 0.7;
  }
  else if (assist_mode == 3)
  {
    mode = "Gravity Compensation";
    //arm_abduction_L = -imu.LTx;
    //arm_flexion_L = -imu.LTz;

    //arm_abduction_R = -imu.RTx;
    //arm_flexion_R = imu.RTz;
    //int_rotation_R = imu.RTy;

    // compute arm elevation from IMU angle components
    arm_elevation_L = cos(imu.LTx*PI/180)*cos(imu.LTy*PI/180);
    arm_elevation_L = acos(arm_elevation_L)*180/PI;
    
    arm_elevation_R = cos(imu.RTx*PI/180)*cos(imu.RTy*PI/180);
    arm_elevation_R = acos(arm_elevation_R)*180/PI;


    if (arm_elevation_L > angle_threshold)
    {
      torque_bio_L = Compute_Biological_Torque(arm_elevation_L);
      moment_arm_L = Compute_moment_arm(arm_elevation_L) / 1000;
      motor_torque_L = torque_bio_L * radius_motor_pulley / moment_arm_L;
      M1_torque_command = Gain_L * Gain_common * motor_torque_L;
    }
    else
    {
      M1_torque_command = 0;
    }
    
    if (arm_elevation_R > angle_threshold)
    {
      torque_bio_R = Compute_Biological_Torque(arm_elevation_R);
      moment_arm_R = Compute_moment_arm(arm_elevation_R) / 1000;
      motor_torque_R = torque_bio_R * radius_motor_pulley / moment_arm_R;
      M2_torque_command = Gain_R * Gain_common * motor_torque_R;
    }
    else
    {
      M2_torque_command = 0;
    }
    
  }
  else
  {
    mode = "Zero Torque";
    M1_torque_command = 0;
    M2_torque_command = 0;
  }
}

float Compute_Biological_Torque(float elevation_angle)
{
  float torque = (ua_weight*ua_moment_arm + fa_weight*fa_moment_arm + h_weight*h_moment_arm) * 9.81 * sin(elevation_angle*PI/180);
  return torque;
}

float Compute_moment_arm(float elevation_angle)
{ 
  float moment_arm = a1*sin(b1*elevation_angle + c1) + a2*sin(b2*elevation_angle + c2);
  return moment_arm;
}

//// initialize motor //// 
void initial_omni_motor() 
{  
  omni_m1.error_clear();    
  delay(200); 
  omni_m2.error_clear();    
  delay(200);  

  omni_m1.omni_torque_ctl_mode_start();    
  delay(200);   
  omni_m2.omni_torque_ctl_mode_start();        
  delay(200);  
  
  omni_m1.omni_motor_start();    
  omni_m1.request_pos_vel();    
  delay(500);   

  omni_m2.omni_motor_start();    
  omni_m2.request_pos_vel();     
  delay(500);    

  //  omni_m1.omni_torque_cmd(0.2);    
  //  omni_m1.request_torque();   
  // delay(200);    
  // omni_m2.omni_torque_cmd(0.2);    
  // omni_m2.request_torque();   
  // delay(200);   
  
  for (int i=0; i < 10; i++)
  {
    receive_torque_ctl_feedback();  //read motor CAN info   
  }
  delay(1000);   

  initial_pos_1 = omni_m1.pos;     
  initial_pos_2 = omni_m2.pos;     

  delay(500);  

  /////// initial command setting ///////
  M1_torque_command = 0.0;         
  M2_torque_command = 0.0;         
}

double clip_torque(double torque_command)
{
  float actual_command = 0.0;  
  actual_command = fminf(fmaxf(MIN_torque_command, torque_command), MAX_torque_command);     
  return actual_command;   
}

void initial_CAN() 
{
  Can3.begin();
  Can3.setBaudRate(1000000);  
  delay(400);  
  Serial.println("Can bus setup done...");  
  delay(200);  
}   

//Receive CAN info from the motor
void receive_torque_ctl_feedback() 
{
  if (Can3.read(msgR))
  //while (Can3.available() > 0)
  {
    Can3.read(msgR);    
    //Serial.print("msgR: "); Serial.println(msgR);  
    //Serial.print("msgR.id: "); Serial.println(msgR.id);  
    if (msgR.id == (omni_motor_ID_1<<5) + 0x009)
    {
      omni_m1.unpack_pos_vel(msgR, initial_pos_1); 
    } 

    if (msgR.id == (omni_motor_ID_1<<5) + 0x01C) 
    {
      omni_m1.unpack_torque(msgR);
      tau_t_1 = omni_m1.torque;       
    }  

    if (msgR.id == (omni_motor_ID_2<<5) + 0x009)  //0x029
    {
      omni_m2.unpack_pos_vel(msgR, initial_pos_2);       
    } 

    if (msgR.id == (omni_motor_ID_2<<5) + 0x01C)   //0x03C
    {
      omni_m2.unpack_torque(msgR);         
      tau_t_2 = omni_m2.torque;           
    }  
  }
}   

//Receive bluetooth data
void Receive_ble_Data(){
  if (Serial5.available() >= 20) {
    // Read the incoming byte:
    Serial5.readBytes(&data_rs232_rx[0], 20);
    
    if (data_rs232_rx[0] == 165) { // Check the first byte

      if (data_rs232_rx[1] == 90) { // Check the second byte

        if (data_rs232_rx[2] == 20) { // Check the number of elemnts in the package

          /*Example bluetooth code
          Rescaling_gain   = ((int16_t)(data_rs232_rx[3] | (data_rs232_rx[4] << 8))) / 100.0;
          Flex_Assist_gain = ((int16_t)(data_rs232_rx[5] | (data_rs232_rx[6] << 8))) / 100.0;
          Ext_Assist_gain  = ((int16_t)(data_rs232_rx[7] | (data_rs232_rx[8] << 8))) / 100.0;
          Assist_delay_gain = data_rs232_rx[9];

          Serial.print(" | ");
          Serial.print(Rescaling_gain);
          Serial.print(" | ");
          Serial.print(Flex_Assist_gain);
          Serial.print(" | ");
          Serial.print(Ext_Assist_gain);
          Serial.print(" | ");
          Serial.print(Assist_delay_gain);
          Serial.println(" | ");
          */
        }
      }
    }
  }
}


void Transmit_ble_Data() {
  t_teensy        = t * 100;
  L_motor_torque  = omni_m1.torque * 100;
  R_motor_torque  = omni_m2.torque * 100;
  L_motor_torque_command = M1_torque_command *100;
  R_motor_torque_command = M2_torque_command *100;

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0]  = 165;
  data_ble[1]  = 90;
  data_ble[2]  = datalength_ble;
  data_ble[3]  = t_teensy;
  data_ble[4]  = t_teensy >> 8;
  data_ble[5]  = 0;
  data_ble[6]  = 0 >> 8;
  data_ble[7]  = 0;
  data_ble[8]  = 0 >> 8;
  data_ble[9]  = L_motor_torque;
  data_ble[10] = L_motor_torque >> 8;
  data_ble[11] = R_motor_torque;
  data_ble[12] = R_motor_torque >> 8;
  data_ble[13] = L_motor_torque_command;
  data_ble[14] = L_motor_torque_command >> 8;
  data_ble[15] = R_motor_torque_command;
  data_ble[16] = R_motor_torque_command >> 8;
  data_ble[17] = 0;
  data_ble[18] = 0 >> 8;
  data_ble[19] = 0;
  data_ble[20] = 0 >> 8;
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 0 >> 8;

  Serial5.write(data_ble, datalength_ble);
}


void print_data() {
  //M1 is left, M2 is right
  Serial.print(current_time);
  Serial.print(" ; angle_1 : "); Serial.print(arm_elevation_L);
  Serial.print(" ; angle_2 : "); Serial.print(arm_elevation_R); 
  Serial.print(" ; M1_cmd : "); Serial.print(M1_torque_command);
  Serial.print(" ; M1_tor : "); Serial.print(omni_m1.torque);    
  Serial.print(" ; M2_cmd : "); Serial.print(M2_torque_command);
  Serial.print(" ; M2_tor : "); Serial.print(omni_m2.torque);  
  Serial.print(" ; M1_pos : "); Serial.print(omni_m1.pos);
  Serial.print(" ; M2_pos : "); Serial.print(omni_m2.pos);
  Serial.println();
}  
