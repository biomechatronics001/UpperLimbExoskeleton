// #include <variant.h>
#include <Arduino.h>

//#define PROFILE_SELECTOR (0)

// SERIAL_WL for wireless serial
// Yellow 3.3V    White RX    Red TX    Black GROUND
#define SERIAL_WL (Serial2)
#define INIT_TIME (4) // unit : second

typedef float CAL_TYPE;

class EncoderCl
{
public:
  void INIT();
//  void READ();
  void INIT_MEAN();
  void UpdateData(double Angle1,double Angle2);
  void Print();
  void READ();
  

  CAL_TYPE KneeAngle[2]={0};
  CAL_TYPE init_RKx=0;
  CAL_TYPE init_LKx=0;
  
  CAL_TYPE LKx=0; 
  CAL_TYPE RKx=0;
  CAL_TYPE RLKx=0;
  CAL_TYPE LKx_filtered_last=0;
  CAL_TYPE LKx_filtered=0;
  CAL_TYPE RKx_filtered_last=0;
  CAL_TYPE RKx_filtered=0;
  CAL_TYPE RLKx_filtered=0;
  CAL_TYPE DOFC[2];
//  CAL_TYPE SquatTorque;
  
  CAL_TYPE y_delay[100]={0};
  CAL_TYPE RLKx_delay[100]={0};
   
  int delayindex=0;
  int delaypoint=50;
  int currentpoint=0;
  int doi=0;
  double Gain_E=1;
  double Gain_F=1;
  double test=0;
  double test1=0;
  double test2=0;
  
  double y_raw=0;
  double y_filtered=0;
  double y_filtered_last=0;

  void DelayedOutputTorqueControl();
  void SquatTorqueCommand();
  
private:

  int count1 = 0;
  uint8_t st = 0;
  uint8_t Datain[203];
  int read_count = 0;
  uint8_t ch;
  void Packet_Decode(uint8_t c);

};
