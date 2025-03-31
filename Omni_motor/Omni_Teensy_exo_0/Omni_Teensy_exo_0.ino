//To make this work: Turn on motors. Load to Teensy while subject standing still. When done (data updating in Serial Monitor)

#include "Serial_Com.h"
#include <Arduino.h>
#include "MovingAverage.h"
#include <math.h>  
#include <iomanip> 
#include <cstring>  

/*MOTOR*/ 
#include <FlexCAN_T4.h>   
#include "Omni_motor_Control.h"

#include "WL_IMU.h"
// #include "ads1292r.h"   

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;   
 
/*Filter for external sensors*/
MovingAverage LTAVx(12);    
MovingAverage RTAVx(12);     
float f_LTAVx = 0;  
float f_RTAVx = 0;  

CAN_message_t msgR;   
int CAN_ID = 3;  

///////////////////////////////////////////////////////

char user_sex = 'M'; // M for male, F for female
float user_weight = 70; // [kg]
float user_height = 1.75; // [m]

int assist_mode = 1;//3;
String mode = "start";

double Gain_L = 1;
double Gain_R = 1;
double Gain_common = 1;

int current_limitation = 5;  //(unit Amp)
int max_allowed_torque = 30; // Safety measurement to limit the commanded torque

int Stop_button = 0;    // Stop function

double Left_Torque_Command;
double Right_Torque_Command;

///////////////////////////////////////////////////////

int Omni_Motor_ID_1 = 1;     
int Omni_Motor_ID_2 = 0;       

double torque_command = 0;  
double velocity_command = 0;  
double position_command = 0;  

double M1_torque_command = 0;  
double M2_torque_command = 0;   

double RL_torque_command_1 = 0.0;    
double RL_torque_command_2 = 0.0;   

double MAX_torque_command = 12;   
double MIN_torque_command = -12;    

int LimitInf = -18;    
int LimitSup = 18;    

float p_des = 0;   
float v_des = 0;    
float kp = 0;   
float kd = 0;   
float t_ff = 0;   

/*MOTOR*/    
float initial_pos_1 = 0;       
float initial_pos_2 = 0;       

Motor_Control_Tmotor omni_m1(0x000, CAN_ID);   
Motor_Control_Tmotor omni_m2(0x001, CAN_ID);   
/*MOTOR*/  

IMU imu;                                                      //Create IMU object see WL_IMU.h

/*Serial Class Setup*/  
Serial_Com Serial_Com;   

// sensor  
// ads1292r torque_sensor1;    //FOR THE TORQUE SENSORS

/*Serial Send*/  
size_t Send_Length = 11; 
char Send[11] = { 0x31, 0x32, 0x32, 0x33, 0x33,
                  0x30, 0x31, 0x32, 0x33, 0x33,
                  0x33 };   

/* Time control*/
double t;   
double next_t;    
double delta_t;    

//***For managing the Controller and Bluetooth rate
unsigned long t_0 = 0;
// double cyclespersec_ctrl = 28;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ctrl = 100;    // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ble  = 20;     // [Hz] Bluetooth sending data frequency 
unsigned long current_time = 0;
unsigned long previous_time = 0;                                           // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                       // used to control the Bluetooth communication frequency
unsigned long Tinterval_ctrl_micros = (unsigned long)(1000000 / cyclespersec_ctrl); // used to control the teensy controller frequency
unsigned long Tinterval_ble_micros  = (unsigned long)(1000000 / cyclespersec_ble);  // used to control the Bluetooth communication frequency
//**********************************

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy

int L_motor_torque  = 0;   
int R_motor_torque  = 0;  
int L_motor_torque_desired = 0;    
int R_motor_torque_desired = 0;   
int t_teensy = 0;  
int M_Selected = 0;  
int CtrlMode_Selected = 0;  

int GUI_force_cmd     = 0;  
int GUI_stiffness_cmd = 0;  
int GUI_damping_cmd   = 0;  
int GUI_assistive_ratio_cmd = 0;     

int GUI_pos_ampl_cmd    = 0;       
int GUI_pos_fre_cmd     = 0;       
int GUI_force_ampl_cmd  = 10;        
int GUI_force_fre_cmd   = 10;       

double GUI_force = 1.0;    
double GUI_K_p   = 1.0;    
double GUI_K_d   = 0.1;    

double assistive_ratio = 0.08;   

int L_pos_int_d = 0;     
int L_pos_int_a = 0;     
int L_vel_int_d = 0;       
int L_vel_int_a = 0;      
 
int R_pos_int_d = 0;     
int R_pos_int_a = 0;       
int R_vel_int_d = 0;         
int R_vel_int_a = 0;       
//**************************

double cmd_ampl = 1.0;        
double cmd_fre  = 1.0;      
float pos_ampl  = 0.0;      
float pos_fre   = 0.5;      

float l_pos_des = 0.0;    
float l_vel_des = 0.0;    
float r_pos_des = 0.0;     
float r_vel_des = 0.0;     

float ref_force_ampl = 0.2;    
float ref_force_fre = 0.5;    

float l_ref_tau    = 0.0;   
float l_ref_tau_dt = 0.0;    
float r_ref_tau    = 0.0;     
float r_ref_tau_dt = 0.0;    

float l_leg_angle    = 0.0;  
float r_leg_angle    = 0.0;  
float l_leg_velocity = 0.0;   
float r_leg_velocity = 0.0;  

//***Impedance Control Test***//  
float tau_imp = 0.0;      
float kp_imp = 1.0;     
float kd_imp = 0.01 * kp_imp;         
//***Impedance Control Test***//    

//***Torque Control Test */
float dt = 0.01;    

float tau_t_1 = 0.0;  
float tau_t_1_last = 0.0;    
float tau_t_2 = 0.0;    
float tau_t_2_last = 0.0;   

float tau_dt_1 = 0.0;    
float tau_dt_2 = 0.0;     

float tau_ff_1 = 0.0;      
float tau_ff_2 = 0.0;   

//*** Motor Mode Set ***//   
//int ctl_method = 1;    // 0 for using custom controller, 1 for using other normal controller  
int ctl_mode = 0;      // 0 for torque control, 1 for mit control    
//int ctl_type = 0;      // 0 for motion, 1 for force tracking, 2 for direct torque   
  
int l_ctl_dir = 1;      
int r_ctl_dir = -1;    
float torque_cmd_scale = 20.0;  

//*** Motor Mode Set ***//    
int doi          = 0;   
int currentpoint = 0;    
int delayindex   = 0;    

int L_motor_torque_command = 0;
int R_motor_torque_command = 0;
double Rescaling_gain    = 0;
double Flex_Assist_gain  = 0;
double Ext_Assist_gain   = 0;
double Assist_delay_gain = 0;  

double S_torque_command_left = 0.0;    
double S_torque_command_right = 0.0;   

////////////////////////////////////////
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
////////////////////////////////////////

//// setup can and motors ////
void setup() {
  delay(3000);   

  Serial.begin(115200);     //115200/9600=12
  Serial2.begin(115200);    //115200/9600=12
  //Serial7.begin(115200);  // Communication with Raspberry PI or PC for High-lever controllers like RL
  Serial5.begin(115200);    //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  
  Serial_Com.INIT();    

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

  initial_CAN();    
  initial_omni_motor();    
  delay(100);

  t_0 = micros();
}  

void loop() {
    Serial_Com.READ2();  
    imu.READ();     

    current_time = micros() - t_0;            
    t = current_time/1000000.0;          
    
    if (current_time - previous_time > Tinterval_ctrl_micros) {
      if (current_time - previous_time_ble > Tinterval_ble_micros) {
        
        //for bluetooth connection
        Receive_ble_Data();  
        Transmit_ble_Data();      
        previous_time_ble = current_time;   
      }  

      currentpoint = doi;  
      delayindex   = doi - Assist_delay_gain;    

      if (delayindex < 0) {
        delayindex = delayindex + 100;   
      }
      else if (delayindex >= 100) {
        delayindex = delayindex - 100;   
      }

      doi++;
      doi = doi % 100;

      //sample sine wave command
      //M1_torque_command = 0.2;//*sin(2*t*PI);
      //M2_torque_command = 0.5*sin(2*t*PI);
      if (Stop_button) //stop
        {
          M1_torque_command = 0;//+0.5;
          M2_torque_command = 0;//-0.5;
        }
      else
        {
          Compute_Torque_Commands(); // Vahid
        }
      
      // clip the torque command  
      M1_torque_command = clip_torque(M1_torque_command);         
      M2_torque_command = clip_torque(M2_torque_command);           
    
      M1_torque_command = M1_torque_command * l_ctl_dir;   
      M2_torque_command = M2_torque_command * r_ctl_dir;        /// for right.   
      
      print_data_motor();
      if (ctl_mode == 1)    
      {
        // mit control    
        omni_m1.omni_mit_ctl_cmd(0.0, 0.0, 10.0, 0.01, M1_torque_command);      
        omni_m2.omni_mit_ctl_cmd(0.0, 0.0, 10.0, 0.01, M2_torque_command);       
        receive_mit_ctl_feedback();      
      } 

      //custom controls
      else 
      {

        for (int i =0; i < 4; i++)   
        {
          //read motor CAN info
          receive_torque_ctl_feedback();     
        }      

        omni_m1.sig_torque_cmd(M1_torque_command);      
        omni_m2.sig_torque_cmd(M2_torque_command);        
      }
      //Wait(2200);
      previous_time = current_time;   
    }  
}  

void Compute_Torque_Commands()
{
  if (assist_mode == 1)
  {
    mode = "Constant Signal";
    M1_torque_command = l_ctl_dir * Gain_L * Gain_common * 0.3;
    M2_torque_command = r_ctl_dir * Gain_R * Gain_common * 0.3;
  }
  else if (assist_mode == 2)
  {
    mode = "Sine Wave";
    M1_torque_command =  Gain_L * sin(2 * PI * t) * 0.3;  // [Amp]
    M2_torque_command =  Gain_R * sin(2 * PI * t) * 0.3;  // [Amp]
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
    //int_rotation_R = imu.RTy;
    //abduction_R = -imu.RTx;
    //flexion_R = imu.RTz;

    if (arm_elevation_L > angle_threshold)
    {
      torque_bio_L = Compute_Biological_Torque(arm_elevation_L);
      moment_arm_L = Compute_moment_arm(arm_elevation_L)/1000;
      motor_torque_L = torque_bio_L*radius_motor_pulley/moment_arm_L;
      M1_torque_command = l_ctl_dir * Gain_L*Gain_common*motor_torque_L;
    }
    else
    {
      M1_torque_command = 0;
    }
    
    if (arm_elevation_R > angle_threshold)
    {
      torque_bio_R = Compute_Biological_Torque(arm_elevation_R);
      moment_arm_R = Compute_moment_arm(arm_elevation_R)/1000;
      motor_torque_R = torque_bio_R*radius_motor_pulley/moment_arm_R;
      M2_torque_command = r_ctl_dir * Gain_R*Gain_common*motor_torque_R;
    }
    else
    {
      M2_torque_command = 0;
    }
    
  }
  else if (assist_mode == 4)
  {
  }
  else if (assist_mode == 5) //Serial communication with High level system
  {
    mode = "High Level Control";
    //send_serial_Data_Highlevel(); 
    //receive_serial_Data_Highlevel();
    M1_torque_command = l_ctl_dir * Left_Torque_Command; //2.2;
    M2_torque_command = r_ctl_dir * Right_Torque_Command; //2.2;
  }
  else if (assist_mode == 100)
  {
    mode = "Zero Current";
    M1_torque_command = 0;
    M2_torque_command = 0;
  }
}

float Compute_Biological_Torque(float elevation_angle)
{
  float torque = (ua_weight*ua_moment_arm + fa_weight*fa_moment_arm + h_weight*h_moment_arm)*9.81*sin(elevation_angle*PI/180);
  return torque;
}

float Compute_moment_arm(float elevation_angle)
{ 
  float moment_arm = a1*sin(b1*elevation_angle + c1) + a2*sin(b2*elevation_angle + c2);
  return moment_arm;
}

//// initial motor //// 
void initial_omni_motor() {  
  omni_m1.error_clear();    
  delay(200); 

  /////////// set control mode /////////  
  if (ctl_mode == 1)  
  {
    omni_m1.omni_mit_ctl_mode_start();      
    omni_m2.omni_mit_ctl_mode_start();     
  } 
  else
  {
    omni_m1.sig_torque_ctl_mode_start();    
    delay(200);   
    omni_m2.sig_torque_ctl_mode_start();        
  } 
  delay(200);  
  
  omni_m1.omni_motor_start();    
  omni_m1.request_pos_vel();    
  delay(500);   

  omni_m2.omni_motor_start();    
  omni_m2.request_pos_vel();     
  delay(500);    

  if (ctl_mode == 1)  
  {
    omni_m1.omni_mit_ctl_cmd(0.0, 0.0, 0.0, 0.0, 0.01);     
    omni_m2.omni_mit_ctl_cmd(0.0, 0.0, 0.0, 0.0, 0.01);          
    receive_mit_ctl_feedback();     
  }
  else{
    // omni_m1.request_torque();   
    omni_m1.sig_torque_cmd(0.01);    
    delay(200);    
    // omni_m2.request_torque();   
    omni_m2.sig_torque_cmd(0.01);      
    delay(200);   
  } 

  for (int i =0; i < 1000; i++)
  {
    receive_torque_ctl_feedback();     
  }
  delay(1000);   

  initial_pos_1 = omni_m1.pos;     
  initial_pos_2 = omni_m2.pos;     

  delay(500);  

  /////// command initial setting ///////
  M1_torque_command = 0.0;         
  M2_torque_command = 0.0;         
}

double clip_torque(double torque_command)
{
  float actual_command = 0.0;  
  actual_command = fminf(fmaxf(MIN_torque_command, torque_command), MAX_torque_command);     

  return actual_command;   
}

void initial_CAN() {
  Can3.begin();
  // Can3.setBaudRate(1000000);  
  Can3.setBaudRate(1000000);  
  delay(400);  
  Serial.println("Can bus setup done...");  
  delay(200);  
}   

float Sig_torque_control(float force_des, float dt_force_des, float force_t, float dt_force_t, float kp, float kd, float tau_ff)  
{
  float tor_cmd = 0;   

  tor_cmd = kp * (force_des - force_t) + kd * (dt_force_des - dt_force_t) + tau_ff; 

  return tor_cmd;   
}  

float omni_motion_control(float pos_des, float vel_des, float pos_t, float vel_t, float kp, float kd, float tau_ff)  
{
  float pos_ctl_cmd = 0;   

  pos_ctl_cmd = kp * (pos_des - pos_t) + kd * (vel_des - vel_t) + tau_ff; 

  return pos_ctl_cmd;   
}  

//Receive CAN info from the motor in mit control
void receive_mit_ctl_feedback() {
  if (Can3.read(msgR)) {
    Can3.read(msgR);  

    if (msgR.id == 0x008)     
    {
      if (msgR.buf[0] == 0x000)      
      {
        omni_m1.unpack_reply(msgR, initial_pos_1);      
      } 
    } 
  }
}   

//Receive CAN info from the motor
void receive_torque_ctl_feedback() {
  if (Can3.read(msgR)) {
    Can3.read(msgR);      

    if (msgR.id == 0x009)  
    {
      omni_m1.unpack_pos_vel(msgR, initial_pos_1);       
    } 

    if (msgR.id == 0x01C) 
    {
      omni_m1.unpack_torque(msgR);    
      tau_t_1 = omni_m1.torque;          
    }  

    if (msgR.id == 0x029)  
    {
      omni_m2.unpack_pos_vel(msgR, initial_pos_2);       
    } 

    if (msgR.id == 0x03C)   
    {
      omni_m2.unpack_torque(msgR);         
      tau_t_2 = omni_m2.torque;           
    }  
  }
}   

//Receive Cbluetooth data from the motor
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
  data_ble[20] = 0;
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

double derivative(double dt, double derivative_prev[], double *actual_in_ptr, double *prev_in_ptr){
  int i;
  double diff = 0.0, diff_sum = 0.0;
  if (dt != 0.0){
    for (i = 0; i < 3; i++){
      diff_sum += derivative_prev[i];
    }
    diff = (diff_sum + (*actual_in_ptr - *prev_in_ptr) / dt) / (i + 1);
  } else 
    diff = derivative_prev[3];
  return diff;
}

void print_data_motor() {
  //  double v1 = 90;
  //  double v2 = -v1;
  //  Serial.print(v1);
  //  Serial.print("   ");
  Serial.print(current_time);
  Serial.print(" ; ");
  Serial.print(" M1_tor ; "); //M1 is left, M2 is right
  Serial.print(omni_m1.torque);    
  Serial.print(" ; M1_cmd ; ");   
  Serial.print(M1_torque_command);   
  Serial.print(" ; M2_tor ; ");  
  Serial.print(omni_m2.torque);  
  Serial.print(" ; M2_cmd ; ");   
  Serial.print(M2_torque_command);
  Serial.print(" ; M1_pos ; ");
  Serial.print(omni_m1.pos);
   Serial.print(" ; M2_pos ; ");
  Serial.print(omni_m2.pos);
  Serial.println(" ;  ");
}  

void M1_Torque_Impedance_Control_Example(){
    p_des = 0.0;  //dont change this
    v_des = 0.0;  //dont change this
    kp = 0.0;        //dont change this
    kd = 0.0;        //dont change this  

    t_ff = M1_torque_command;   
    tau_imp = (p_des - omni_m1.pos) * kp + (v_des - omni_m1.spe) * kd + t_ff;    

    tau_imp = t_ff; 
    omni_m1.omni_mit_ctl_cmd(p_des, v_des, kp, kd, tau_imp);   

    receive_mit_ctl_feedback();  
} 

void omni_m_Torque_Impedance_Control_Example(){ 
    p_des = 0.0;  //dont change this    
    v_des = 0.0;  //dont change this    
    kp = 0.0;     //dont change this    
    kd = 0.0;     //dont change this     

    t_ff = M1_torque_command;   

    tau_imp = t_ff;   
    omni_m1.omni_mit_ctl_cmd(p_des, v_des, kp, kd, tau_imp);    

    receive_mit_ctl_feedback();    
} 

void M2_Torque_Impedance_Control_Example(){
    p_des = 0.0;      //dont change this
    v_des = 0.0;  //dont change this
    kp = 0.0;        //dont change this
    kd = 0.0;        //dont change this  
 
    t_ff = 0.3;     
    tau_imp = t_ff;    
    omni_m2.omni_mit_ctl_cmd(p_des, v_des, kp, kd, tau_imp);    

    receive_mit_ctl_feedback();    
}  

void M1_Position_Control_Example() {
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0;  //dont change this
  kp = 30;    //max 450 min 0
  kd = 1.5;   //max 5 min 0
  t_ff = 0;   //dont change this
  omni_m1.omni_mit_ctl_cmd(p_des, v_des, kp, kd, t_ff);
  receive_mit_ctl_feedback();
}

void M2_Position_Control_Example() {
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0;  //dont change this
  kp = 30;    //max 450 min 0
  kd = 1.5;   //max 5 min 0
  t_ff = 0;   //dont change this
  omni_m2.omni_mit_ctl_cmd(p_des, v_des, kp, kd, t_ff);
  receive_mit_ctl_feedback();
}  
