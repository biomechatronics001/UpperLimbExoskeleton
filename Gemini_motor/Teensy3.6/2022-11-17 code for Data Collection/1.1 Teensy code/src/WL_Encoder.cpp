#include "WL_Encoder.h"


void EncoderCl::UpdateData(double Angle1,double Angle2)
{
  KneeAngle[0] = -Angle1; 
  KneeAngle[1] = Angle2;
}

void EncoderCl::DelayedOutputTorqueControl()
{
  LKx = KneeAngle[0];//init_LKx;
  RKx = KneeAngle[1];//init_RKx;    
  
  RLKx=RKx-LKx;//right-left knee angle difference 
  LKx_filtered_last = LKx_filtered;
  LKx_filtered = ((0.95*LKx_filtered_last) + (0.05*LKx));
  RKx_filtered_last = RKx_filtered;
  RKx_filtered = ((0.95*RKx_filtered_last) + (0.05*RKx));  
  RLKx_filtered = RKx_filtered-LKx_filtered;
//  RLKx_filtered = 60.0;
  y_filtered = (sin(RKx_filtered*PI/180) - sin(LKx_filtered*PI/180));
  y_delay[doi] = y_filtered;
  RLKx_delay[doi] = RLKx_filtered;
  currentpoint = doi; 
  delayindex = doi-delaypoint;
  if (delayindex<0)
  {
    delayindex = delayindex+100;
  }
  doi++;
  
//  Serial.println(doi);
  doi = doi%100;
//  doi = 10;
//////**************Samsung walking algorithm************
     test = -Gain_E*y_delay[delayindex];//left knee torque  
     test1 = -Gain_F*y_delay[delayindex];//left knee torque
     test2 = RLKx_delay[delayindex];
    if( (RLKx_delay[delayindex]>=0) && (RLKx_delay[delayindex]<120)  )
    {  
      DOFC[0] = Gain_E*y_delay[delayindex];//left knee torque  
      DOFC[1] =  -Gain_F*y_delay[delayindex];//right knee torque
    }
    else if((RLKx_delay[delayindex]<0) && (RLKx_delay[delayindex]>-120) )
    {
      DOFC[1] =  -Gain_E*y_delay[delayindex];//right knee torque
      DOFC[0] = Gain_F*y_delay[delayindex];//left knee torque
    }
    else
    {
     // DOFC[1]=0;
     // DOFC[0]=0;
    }
}

// squat torque assistant
//void DOFControl::SquatTorqueCommand()
//{
//  SquatTorque=-0.5*(52.2*(-9.8)*(0.287*(-1)*sin(TKx*PI/180)+0.441*sin((-1)*(RTx+LTx)/2*PI/180))+19.6*(-9.8)*0.245*sin((-1)*(RTx+LTx)/2*PI/180));
//}
