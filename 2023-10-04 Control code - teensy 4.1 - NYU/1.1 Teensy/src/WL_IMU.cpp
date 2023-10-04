#include "WL_IMU.h"
union u_tag
{
  byte b[4];
  float fval;
} u;
float IMUdata[52] = {0};

void IMU::Packet_Decode(uint8_t c)
{
  switch (st)
  {
  case 0: // Read 1st Byte
    if (c == 0x3a)
    {
      st = 1;
      Datain[read_count] = c;
      read_count += 1;
    }
    break;
  case 1: // Read 2nd Byte
    if (c == 0xc4)
    {
      st = 2;
      Datain[read_count] = c;
      read_count += 1;
    }
    else
    {
      st = 0;
      read_count = 0;
    }
    break;
  case 2:
    Datain[read_count] = c;
    read_count += 1;
    if (read_count >= 203)
    {
      st = 0;
      read_count = 0;
    }
    break;
  default:
    st = 0;
    break;
  }
}

void IMU::INIT()
{
  SERIAL_WL.begin(230400);
}

void IMU::READ()
{
  if (SERIAL_WL.available())
  {
    ch = SERIAL_WL.read();
    Packet_Decode(ch);
  }
}

void IMU::INIT_MEAN()
{
  unsigned long timing = micros();
  while (micros() - timing < INIT_TIME * 1000000)
  {
    unsigned long n = 0;
    READ();
    GetData();

    init_TKx += (IMUdata[4] - init_TKx) / (n + 1);
    init_TKy += (IMUdata[5] - init_TKy) / (n + 1);
    init_TKz += (IMUdata[6] - init_TKz) / (n + 1);

    init_LTx += (IMUdata[11] - init_LTx) / (n + 1);
    init_LTy += (IMUdata[12] - init_LTy) / (n + 1);
    init_LTz += (IMUdata[13] - init_LTz) / (n + 1);

    init_RTx += (IMUdata[18] - init_RTx) / (n + 1);
    init_RTy += (IMUdata[19] - init_RTy) / (n + 1);
    init_RTz += (IMUdata[20] - init_RTz) / (n + 1);

    init_LSx += (IMUdata[25] - init_LSx) / (n + 1);
    init_LSy += (IMUdata[26] - init_LSy) / (n + 1);
    init_LSz += (IMUdata[27] - init_LSz) / (n + 1);

    init_RSx += (IMUdata[32] - init_RSx) / (n + 1);
    init_RSy += (IMUdata[33] - init_RSy) / (n + 1);
    init_RSz += (IMUdata[34] - init_RSz) / (n + 1);

    init_LFx += (IMUdata[39] - init_LFx) / (n + 1);
    init_LFy += (IMUdata[40] - init_LFy) / (n + 1);
    init_LFz += (IMUdata[41] - init_LFz) / (n + 1);

    init_RFx += (IMUdata[46] - init_RFx) / (n + 1);
    init_RFy += (IMUdata[47] - init_RFy) / (n + 1);
    init_RFz += (IMUdata[48] - init_RFz) / (n + 1);

    n += 1;
  }
}

void IMU::GetData()
{
  for (int i = 3; i < 198; i = i + 4)
  {
    u.b[0] = Datain[i];
    u.b[1] = Datain[i + 1];
    u.b[2] = Datain[i + 2];
    u.b[3] = Datain[i + 3];
    IMUdata[(i - 3) / 4] = u.fval;
  }
}
