//#ifndef _Motor_Control_H
//#define _Motor_Control_H
#include <FlexCAN.h>
#include <Arduino.h>
#ifndef __MK66FX1M0__
#error "Teensy 3.6 with dual CAN bus is required to run this code"
#endif
#define profile_position_control_mode 6
#define position_control_mode 3
#define velocity_control_mode 2
#define current_control_mode 1
class Motor_Control
{
  public:

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
    int16_t maxiSpeed = 0;
    int16_t iq=0;
    double iq_A=0;
    int16_t iA=0;
    double iA_A=0;
    int16_t iB=0;
    double iB_A=0;
    int16_t iC=0;
    double iC_A=0;
    int16_t speed_value_int16=0;    
    double speed_value=0;        
    uint16_t encoder=0; 
    int8_t temperature=0;    
    uint16_t encoderRaw =0;
    uint16_t encoderOffset =0;
    double motorAngle=0;
    double motorAngle_offset=0;
    double motorAngle_raw=0;
    int64_t motorAngle_int64=0;

    int32_t motorAngle_int32=0;
    uint16_t circleAngle=0;
    uint16_t voltage=0;
    uint8_t errorState=0;

    Motor_Control(uint32_t id, int Can_id,double Gear_ratio);
    ~Motor_Control();
    void init_motor();
    void DataExplanation(CAN_message_t msg2);
    void receive_CAN_data();
    void read_PID();
    void write_PID_RAM();
    void write_PID_ROM();
    void read_acceleration();
    void write_acceleration_RAM();
    void read_encoder();
    void write_encoder_offset_RAM(uint16_t encoder_Offset );
    void write_current_position_as_zero_position();
    void read_multi_turns_angle();
    void read_single_turns_angle();
    void clear_motor_angle_command();
    void read_motor_status();
    void clear_motor_error();
    void read_motor_status_2();
    void read_motor_status_3();
    void close_motor();
    void stop_motor();
    void start_motor();
    void send_current_command(double current);
    void send_speed_command(double speedvalue);
    void send_position_command(double angle);
    void send_position_command_2(double angle, double max_speed);
    void send_position_command_3(double angle, uint8_t spinDirection);
    void send_position_command_4(double angle, double max_speed, uint8_t spinDirection);
    void send_multi_motor_current_command(double Motor1_current, double Motor2_current, double Motor3_current, double Motor4_current);

  private:
    CAN_message_t msgW;
    CAN_message_t msgR;
    struct CAN_filter_t defaultMask;
    uint32_t ID;
    double gear_ratio;
    uint32_t acceleration_uint32=0;
    int32_t acceleration_int32=0;
  
};
//#endif
