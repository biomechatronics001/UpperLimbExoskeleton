
#include <FlexCAN_T4.h>
#include <Arduino.h>

#define profile_position_control_mode 6
#define position_control_mode 3
#define velocity_control_mode 2
#define current_control_mode 1
class Gemini_Teensy41
{
  public:
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
    uint8_t anglePidKp = 10;
    uint8_t anglePidKi = 10;
    uint8_t speedPidKp = 10;
    uint8_t speedPidKi = 10;
    uint8_t iqPidKp = 10;
    uint8_t iqPidKi = 10;

    int32_t Accel = 0;
    int16_t iqControl = 0;
    int16_t iqControl_1 = 0;
    int16_t iqControl_2 = 0;
    int16_t iqControl_3 = 0;
    int16_t iqControl_4 = 0;
    int32_t speedControl = 0;
    int32_t angleControl = 0;

    int16_t iq = 0;
    double iq_A = 0;
    int16_t iA = 0;
    double iA_A = 0;
    int16_t iB = 0;
    double iB_A = 0;
    int16_t iC = 0;
    double iC_A = 0;
    int16_t speed_value_int16 = 0;
    double speed_value = 0;
    int16_t encoder = 0;
    int8_t temperature = 0;
    uint16_t encoderRaw = 0;
    uint16_t encoderOffset = 0;
    double motorAngle = 0;
    double motorAngle_offset = 0;
    double motorAngle_raw = 0;
    int64_t motorAngle_int64 = 0;

    int32_t motorAngle_int32 = 0;
    uint16_t circleAngle = 0;
    uint16_t voltage = 0;
    uint8_t errorState = 0;

    Gemini_Teensy41(uint32_t id, int Can_id, double Gear_ratio, uint16_t Maxspeed_position);
    void init_motor();
    void DataExplanation(CAN_message_t msg2);
    void receive_CAN_data();

    void read_PID();
    void write_PID_RAM();
    void write_PID_ROM();
    void read_acceleration();
    void write_acceleration_RAM();
    void read_multi_turns_angle();
    void send_current_command(double current);
    void send_speed_command(double speedvalue);
    void send_position_command(double angle);

  private:
    CAN_message_t msgW;
    CAN_message_t msgR;
    uint32_t ID;
    double gear_ratio;
    uint16_t maxiSpeed = 0;
    uint32_t acceleration_uint32 = 0;
    int32_t acceleration_int32 = 0;

};
//#endif
