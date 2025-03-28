#include "Gemini_Teensy41.h"

Gemini_Teensy41::Gemini_Teensy41(uint32_t id, int Can_id, double Gear_ratio, uint16_t Maxspeed_position)
{
  ID = id;
  gear_ratio = Gear_ratio;
  maxiSpeed = Maxspeed_position * gear_ratio;
}
void Gemini_Teensy41::init_motor()
{
// ***************** read and set PID for position/velocity/speed parameters *****************//
//  read_PID();
//  delay(100);
//  receive_CAN_data();
//  delay(100);
//
//  Serial.print(anglePidKp);
//  Serial.print("  ");
//  Serial.print(anglePidKi);
//  Serial.print("  ");
//  Serial.print(speedPidKp);
//  Serial.print("  ");
//  Serial.print(speedPidKi);
//  Serial.print("  ");
//  Serial.print(iqPidKp);
//  Serial.print("  ");
//  Serial.println(iqPidKi);

//  anglePidKp = 50;//50 is the default value, 0~255; change Kp to 200 can make it faster
//  anglePidKi = 50;//50 is the default value, 0~255; add a large KI will cause huge vibration
//  speedPidKp = 100;//100 is the default value, 0~255; no obvious difference to increase Kp, it limited by motor acceleration and speed
//  speedPidKi = 5;//5 is the default value, 0~255; add a large KI will cause huge vibration
//  iqPidKp = 100;//100 is the default value, 0~255;
//  iqPidKi = 0;//0 is the default value, 0~255; add KI will cause huge vibration

//  write_PID_ROM();
//  delay(100);
//  receive_CAN_data();
//  delay(100);

//  Serial.print(anglePidKp);
//  Serial.print("  ");
//  Serial.print(anglePidKi);
//  Serial.print("  ");
//  Serial.print(speedPidKp);
//  Serial.print("  ");
//  Serial.print(speedPidKi);
//  Serial.print("  ");
//  Serial.print(iqPidKp);
//  Serial.print("  ");
//  Serial.println(iqPidKi);

  send_current_command(0);
  delay(100);
  receive_CAN_data();
  delay(100);
  send_speed_command(0);//Vel_command_1);
  delay(100);
  receive_CAN_data();
  delay(100);
  read_multi_turns_angle();
  delay(100);
  receive_CAN_data();
  delay(100);
  motorAngle_offset = motorAngle_raw;
}

////////////////Received CAN Message Decoding////////////////////////////////
void Gemini_Teensy41::DataExplanation(CAN_message_t msgR2)
{
  int len = msgR2.len;
  if (len == 8)
  {
    switch (msgR2.buf[0])
    {
      case 0x30://1.Read PID gain
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        break;
      case 0x31: //2.write PID to RAM
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        break;
      case 0x32: //3.write PID to ROM
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        break;
      case 0x33: //4.read Accel
        acceleration_uint32 = (uint32_t)(((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4]));
        acceleration_int32 = (int32_t)acceleration_uint32;
        Accel = acceleration_int32; //unit 1dps/s dps(degree per second)
        break;
      case 0x34: //5.write Accel to RAM
        acceleration_uint32 = (uint32_t)(((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4]));
        acceleration_int32 = (int32_t)acceleration_uint32;
        Accel = acceleration_int32; //unit 1dps/s dps(degree per second)
        break;

      case 0x92: //9: read multi-turn motorAngle (it is int32 datatype ; datasheet is wrong)

        motorAngle_int32 = (int32_t)( ((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4] )) ;
        motorAngle_int32 = motorAngle_int32 * 0.01 ;
        motorAngle_raw = ((double) motorAngle_int32);
        motorAngle = motorAngle_raw - motorAngle_offset;
        break;

      case 0xA1: //19: send current command and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) / 100;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (int16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        if (encoder > 32767)
        {
          encoder = encoder - 65536;
        }
        break;

      case 0xA2: //20: send speed command and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) / 100;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (int16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        if (encoder > 32767)
        {
          encoder = encoder - 65536;
        }
        break;

      case 0xA4: //22: send position command 2(mulitturn command) and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) / 100;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (int16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        if (encoder > 32767)
        {
          encoder = encoder - 65536;
        }
        break;
    }
  }
}
/////////////////////////////////////////////////////////////////////
//******1.Read PID data command******//
void Gemini_Teensy41::read_PID()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x30;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//******2.Write PID gain to RAM******//
void Gemini_Teensy41::write_PID_RAM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x31;
  msgW.buf[1] = 0;
  msgW.buf[2] = anglePidKp;
  msgW.buf[3] = anglePidKi;
  msgW.buf[4] = speedPidKp;
  msgW.buf[5] = speedPidKi;
  msgW.buf[6] = iqPidKp;
  msgW.buf[7] = iqPidKi;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******3.Write PID gain to ROM******//
void Gemini_Teensy41::write_PID_ROM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x32;
  msgW.buf[1] = 0;
  msgW.buf[2] = anglePidKp;
  msgW.buf[3] = anglePidKi;
  msgW.buf[4] = speedPidKp;
  msgW.buf[5] = speedPidKi;
  msgW.buf[6] = iqPidKp;
  msgW.buf[7] = iqPidKi;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******4.Read Acceleration******//
void Gemini_Teensy41::read_acceleration()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x33;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//******5.Write Acceleration RAM******//
void Gemini_Teensy41::write_acceleration_RAM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x34;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&Accel);
  msgW.buf[5] = *((uint8_t*)(&Accel) + 1);
  msgW.buf[6] = *((uint8_t*)(&Accel) + 2);
  msgW.buf[7] = *((uint8_t*)(&Accel) + 3);
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******9.Read multi turns angle command******//
void Gemini_Teensy41::read_multi_turns_angle()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.buf[0] = 0x92;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //delay(1);
  //receive_CAN_data();
}

//******19.current control: send current command current unit A(not limited by maximum Torque Current)******//
void Gemini_Teensy41::send_current_command(double current)
{
  current = current * 100;
  iqControl = (int16_t)current;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.buf[0] = 0xA1;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&iqControl);
  msgW.buf[5] = *((uint8_t*)(&iqControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
}

//******20.speed control: send speed command speed unit dps******//
void Gemini_Teensy41::send_speed_command(double speedvalue)
{
  speedvalue = speedvalue * 100 * gear_ratio;
  speedControl = (int32_t)speedvalue;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.buf[0] = 0xA2;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&speedControl);
  msgW.buf[5] = *((uint8_t*)(&speedControl) + 1);
  msgW.buf[6] = *((uint8_t*)(&speedControl) + 2);
  msgW.buf[7] = *((uint8_t*)(&speedControl) + 3);
  Can3.write(msgW);
}

//******21.position control:send position command (angle unit degree)******//
void Gemini_Teensy41::send_position_command(double angle)
{
  angle = angle * gear_ratio * 100;
  angleControl = (int32_t)angle;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.buf[0] = 0xA4;
  msgW.buf[1] = 0x00;
  msgW.buf[2] = *(uint8_t*)(&maxiSpeed);
  msgW.buf[3] = *((uint8_t*)(&maxiSpeed) + 1);
  msgW.buf[4] = *(uint8_t*)(&angleControl);
  msgW.buf[5] = *((uint8_t*)(&angleControl) + 1);
  msgW.buf[6] = *((uint8_t*)(&angleControl) + 2);
  msgW.buf[7] = *((uint8_t*)(&angleControl) + 3);
  Can3.write(msgW);
}

void Gemini_Teensy41::receive_CAN_data()
{
  Can3.read(msgR);
  DataExplanation(msgR);
}
