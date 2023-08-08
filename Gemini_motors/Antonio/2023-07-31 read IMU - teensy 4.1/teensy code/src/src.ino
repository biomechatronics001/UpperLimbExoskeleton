#include <SPI.h>
#include "WL_IMU.h"
#include <Arduino.h>
#include <stdio.h>

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

int assist_mode = 1;
String mode = "start";

double Gear_ratio = 6; //The actuator gear ratio, will enfluence actuator angle and angular velocity
double Torque_const = 1.67; // Torque constant already takes into account the gear ratio

double Gain_L = 1;
double Gain_R = 1;
double Gain_common = 0;

int Stop_button = 0;    // Stop function
int saveDataFlag = 1;

double weight = 70; // [kg] weight of the subject

uint32_t Motor_ID1 = 1; // Motor Can Bus ID, left leg, loadcell: port 1, positive current = extension // updated on 2022-04-01 2
uint32_t Motor_ID2 = 2; // Motor Can Bus ID, right leg, loadcell: port 2, positive current = flexion // updated on 2022-04-01 3
int CAN_ID = 0;         // CAN port from Teensy

const float p1 = -1.027e-09;
const float p2 =  2.757e-07;
const float p3 = -3.17e-05;
const float p4 =  0.004151;
const float p5 = -0.04245;

float arm_elevation_L;
float arm_elevation_R;

IMU imu;                                                      //Create IMU object see WL_IMU.h

//double Fsample = 500;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
double Fsample = 10;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)

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

//double elevationY = 0;
//double elevationY2 = 0;
double elevationZ = 0;


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

int LK_ble = 0;                //left knee angle
int RK_ble = 0;                //right knee angle
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
  
  // torque_sensor1.Torque_sensor_initial();                                           //initial the torque sensor see ads1292r.cpp.
  // torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 2, 0.0003446 * (-1) * 2.35); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.
  // torque_sensor1.Torque_sensor_offset_calibration();                                //Auto offset the torque sensor to zero. see ads1292r.cpp.
  
  delay(500);
  
  imu.Gain_E = 1;         //Extension gain for delay output feedback control
  imu.Gain_F = 1;         //Flexion gain for delay output feedback control  DOFC.Gain_E = 10;            //Extension gain for delay output feedback control
  imu.STS_Gain = 0;
  imu.delaypoint = 0;     //realative to delay time (delaypoint*sampletime=delaytime) for delay output feedback control
  imu.alpha = 5;
  imu.beta = 2;
  imu.angleRelativeThreshold = 20;

  imu.INIT(); //Initialize IMU;
  delay(500);
  imu.INIT_MEAN();
  current_time = micros();
  previous_time = current_time;
  previous_time_ble = current_time;
}

void loop()
{
  while (stopFlag)
  {  };
  CurrentControl();
}

void CurrentControl()
{
  imu.READ();                          //Check if IMU data available and read it. the sample rate is 100 hz
  current_time = micros();             //query current time (microsencond)

  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    //Serial.print(current_time/1e6); Serial.print("   ");    
    if (isLogging)
    {
      logData3();
    }
    previous_time = current_time; //reset previous control loop time
    relTime += Tinterval_microsecond / 1000;
    print_Data();
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

void print_Data()
{
  Serial.print(imu.RTx); Serial.print(" ");
  Serial.print(imu.RTy); Serial.print(" ");
  Serial.print(imu.RTz); Serial.print(" ");
  //elevationY = cos(imu.RTx*PI/180)*cos(imu.RTz*PI/180) - sin(imu.RTx*PI/180)*sin(imu.RTz*PI/180)*sin(imu.RTy*PI/180);
  //elevationY = acos(elevationY)*180/PI;
  //Serial.print(elevationY); Serial.print(" ");
  elevationZ = cos(imu.RTx*PI/180)*cos(imu.RTy*PI/180);
  elevationZ = acos(elevationZ)*180/PI;
  Serial.print(elevationZ); Serial.print(" ");

  //elevationY2 = cos(imu.RTx*PI/180)*cos(imu.RTz*PI/180) + sin(imu.RTx*PI/180)*sin(imu.RTz*PI/180)*sin(imu.RTy*PI/180);
  //elevationY2 = acos(elevationY2)*180/PI;
  //Serial.print(elevationY2); Serial.print(" ");
//  Serial.print(m1.speed_value); Serial.print("  "); // angle velocity unit: degree per second
//  Serial.print(m2.speed_value); Serial.print("  "); // angle velocity unit: degree per second
  Serial.println();
}
