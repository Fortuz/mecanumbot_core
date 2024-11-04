#ifndef MECANUMBOT_GRABBER_H_
#define MECANUMBOT_GRABBER_H_

#include <Dynamixel2Arduino.h>

#define TORQUE_ENABLE ControlTableItem::TORQUE_ENABLE

enum GrabberMotorLocation{
  LEFT = 0,
  RIGHT,
  GRABBER_MOTOR_NUM_MAX
};

class MecanumbotGrabber
{
public:
    MecanumbotGrabber();
    ~MecanumbotGrabber();
    
    bool init(void);
    void close(void);

    bool is_connected();

    bool set_torque(bool onoff);
    bool get_torque();
    
    bool read_present_position(int32_t &left_value, int32_t &right_value);
    bool read_present_velocity(int32_t &left_value, int32_t &right_value);
    bool read_present_current(int16_t &left_value, int16_t &right_value);
    bool read_profile_acceleration(uint32_t &left_value, uint32_t &right_value);
    
    bool write_position(int16_t left_value, int16_t right_value);
    bool write_profile_acceleration(uint32_t left_value, uint32_t right_value);

    bool control_grabber(int16_t left_grabber_value, int16_t right_grabber_value);

    Dynamixel2Arduino& getDxl();
  
private:
    /* Limit values */
    const int64_t LIMIT_GRABBER_MIN = 160; // 347= straight forward
    const int64_t LIMIT_GRABBER_MAX = 854;

    /* DYNAMIXEL Information for controlling motors */
    const uint8_t DXL_MOTOR_ID_LEFT = 5; // ID of left motor
    const uint8_t DXL_MOTOR_ID_RIGHT = 6; // ID of right motor
    const float DXL_PORT_PROTOCOL_VERSION = 1.0; // Dynamixel protocol version 1.0
    const uint32_t DXL_PORT_BAUDRATE = 1000000; // Baud rate of Dynamixel
    const int OPENCR_DXL_DIR_PIN = 84; // Arduino pin number of DYNAMIXEL direction pin on OpenCR.

    ParamForSyncReadInst_t sync_read_param;
    ParamForSyncWriteInst_t sync_write_param;
    RecvInfoFromStatusInst_t read_result;
    Dynamixel2Arduino dxl;
    uint8_t left_grabber_id_;
    uint8_t right_grabber_id_;
    bool torque_;
};

#endif // MECANUMBOT_GRABBER_H_