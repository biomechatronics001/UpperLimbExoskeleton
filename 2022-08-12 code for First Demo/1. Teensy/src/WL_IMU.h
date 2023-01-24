// #include <variant.h>
#include <Arduino.h>

#define PROFILE_SELECTOR (0)

// SERIAL_WL for wireless serial
// Yellow 3.3V    White RX    Red TX    Black GROUND
#define SERIAL_WL (Serial2)
#define INIT_TIME (4) // unit : second

typedef float CAL_TYPE;

class IMU
{
public:
  void INIT();
  void INIT_MEAN();
  void READ();
  void GetData();
  void Print();

  CAL_TYPE init_TKx=0;
  CAL_TYPE init_TKy=0;
  CAL_TYPE init_TKz=0;
  CAL_TYPE init_LTx=0;
  CAL_TYPE init_LTy=0;
  CAL_TYPE init_LTz=0;
  CAL_TYPE init_RTx=0;
  CAL_TYPE init_RTy=0;
  CAL_TYPE init_RTz=0;
  CAL_TYPE init_LSx=0;
  CAL_TYPE init_LSy=0;
  CAL_TYPE init_LSz=0;
  CAL_TYPE init_RSx=0;
  CAL_TYPE init_RSy=0;
  CAL_TYPE init_RSz=0;
  CAL_TYPE init_LFx=0;
  CAL_TYPE init_LFy=0;
  CAL_TYPE init_LFz=0;
  CAL_TYPE init_RFx=0;
  CAL_TYPE init_RFy=0;
  CAL_TYPE init_RFz=0;

  CAL_TYPE TKx=0;
  CAL_TYPE LTx=0;
  CAL_TYPE RTx=0;
  CAL_TYPE LSx=0;
  CAL_TYPE RSx=0;
  CAL_TYPE LFx=0;
  CAL_TYPE RFx=0;
  CAL_TYPE TKy=0;
  CAL_TYPE LTy=0;
  CAL_TYPE RTy=0;
  CAL_TYPE LSy=0;
  CAL_TYPE RSy=0;
  CAL_TYPE LFy=0;
  CAL_TYPE RFy=0;
  CAL_TYPE TKz=0;
  CAL_TYPE LTz=0;
  CAL_TYPE RTz=0;
  CAL_TYPE LSz=0;
  CAL_TYPE RSz=0;
  CAL_TYPE LFz=0;
  CAL_TYPE RFz=0;

  CAL_TYPE TKAVx=0;
  CAL_TYPE LTAVx=0;
  CAL_TYPE RTAVx=0;
  CAL_TYPE LSAVx=0;
  CAL_TYPE RSAVx=0;
  CAL_TYPE LFAVx=0;
  CAL_TYPE RFAVx=0;
  CAL_TYPE TKAVy=0;
  CAL_TYPE LTAVy=0;
  CAL_TYPE RTAVy=0;
  CAL_TYPE LSAVy=0;
  CAL_TYPE RSAVy=0;
  CAL_TYPE LFAVy=0;
  CAL_TYPE RFAVy=0;
  CAL_TYPE TKAVz=0;
  CAL_TYPE LTAVz=0;
  CAL_TYPE RTAVz=0;
  CAL_TYPE LSAVz=0;
  CAL_TYPE RSAVz=0;
  CAL_TYPE LFAVz=0;
  CAL_TYPE RFAVz=0;
  
  CAL_TYPE LKx=0; 
  CAL_TYPE RKx=0;
  CAL_TYPE RLKx=0;
  CAL_TYPE LKx_filtered_last=0;
  CAL_TYPE LKx_filtered=0;
  CAL_TYPE RKx_filtered_last=0;
  CAL_TYPE RKx_filtered=0;
  CAL_TYPE RLKx_filtered=0;
  CAL_TYPE DOTC[2];
  CAL_TYPE SquatTorque;
  
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

  void DelayOutputTorqueCommand();
  void SquatTorqueCommand();
  
private:

  int count1 = 0;
  uint8_t st = 0;
  uint8_t Datain[203];
  int read_count = 0;
  uint8_t ch;
  void Packet_Decode(uint8_t c);

};
