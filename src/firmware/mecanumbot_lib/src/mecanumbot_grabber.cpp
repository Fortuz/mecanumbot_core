#include "../include/mecanumbot_grabber.h"

MecanumbotGrabber::MecanumbotGrabber()
: dxl(Serial3, OPENCR_DXL_DIR_PIN),
  left_grabber_id_(DXL_MOTOR_ID_LEFT),
  right_grabber_id_(DXL_MOTOR_ID_RIGHT),
  torque_(false)
{

}

MecanumbotGrabber::~MecanumbotGrabber()
{
    close();
    digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

bool MecanumbotGrabber::init(void)
{
    pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
    digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
    drv_dxl_init();

    dxl.begin(DXL_PORT_BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);

    sync_write_param.id_count = 2;
    sync_write_param.xel[LEFT].id = left_grabber_id_;
    sync_write_param.xel[RIGHT].id = right_grabber_id_;

    sync_read_param.addr = 132;
    sync_read_param.length = 4;
    sync_read_param.id_count = 2;
    sync_read_param.xel[LEFT].id = left_grabber_id_;
    sync_read_param.xel[RIGHT].id = right_grabber_id_;

    // Enable Dynamixel Torque
    set_torque(true);

    return true;
}

bool MecanumbotGrabber::is_connected()
{
    return (dxl.ping(DXL_MOTOR_ID_LEFT) == true && dxl.ping(DXL_MOTOR_ID_RIGHT) == true);
}

bool MecanumbotGrabber::set_torque(bool onoff)
{
    bool ret = false;

    sync_write_param.addr = 64;
    sync_write_param.length = 1;
    sync_write_param.xel[LEFT].data[0] = onoff;
    sync_write_param.xel[RIGHT].data[0] = onoff;

    if(dxl.syncWrite(sync_write_param) == true){
      ret = true;
      torque_ = onoff;
    }

    return ret;
}

bool MecanumbotGrabber::get_torque()
{
    if(dxl.readControlTableItem(TORQUE_ENABLE, left_grabber_id_) == true
        && dxl.readControlTableItem(TORQUE_ENABLE, right_grabber_id_) == true){
        torque_ = true;
    }else{
      torque_ = false;
    }

    return torque_;
}

void MecanumbotGrabber::close(void)
{
  // Disable Dynamixel Torque
  set_torque(false);
}

bool MecanumbotGrabber::read_present_position(int32_t &left_value, int32_t &right_value)
{
    bool ret = false;

    sync_read_param.addr = 132;
    sync_read_param.length = 4;

    if(dxl.syncRead(sync_read_param, read_result)){
        memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
        memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
        ret = true;
    }

    return ret;
}

bool MecanumbotGrabber::read_present_velocity(int32_t &left_value, int32_t &right_value)
{
    bool ret = false;

    sync_read_param.addr = 128;
    sync_read_param.length = 4;

    if(dxl.syncRead(sync_read_param, read_result)){
        memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
        memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
        ret = true;
    }

    return ret;
}

bool MecanumbotGrabber::read_present_current(int16_t &left_value, int16_t &right_value)
{
    bool ret = false;

    sync_read_param.addr = 126;
    sync_read_param.length = 2;

    if(dxl.syncRead(sync_read_param, read_result)){
        memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
        memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
        ret = true;
    }

    return ret;
}

bool MecanumbotGrabber::read_profile_acceleration(uint32_t &left_value, uint32_t &right_value)
{
    bool ret = false;

    sync_read_param.addr = 108;
    sync_read_param.length = 4;

    if(dxl.syncRead(sync_read_param, read_result)){
        memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
        memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
        ret = true;
    }

    return ret;
}

bool MecanumbotGrabber::write_position(int16_t left_value, int16_t right_value)
{
    bool ret = false;

    sync_write_param.addr = 30;
    sync_write_param.length = 2;
    memcpy(sync_write_param.xel[LEFT].data, &left_value, sync_write_param.length);
    memcpy(sync_write_param.xel[RIGHT].data, &right_value, sync_write_param.length);

    if(dxl.syncWrite(sync_write_param)){
        ret = true;
    }

    return ret;
}

bool MecanumbotGrabber::write_profile_acceleration(uint32_t left_value, uint32_t right_value)
{
    bool ret = false;

    sync_write_param.addr = 108;
    sync_write_param.length = 4;
    memcpy(sync_write_param.xel[LEFT].data, &left_value, sync_write_param.length);
    memcpy(sync_write_param.xel[RIGHT].data, &right_value, sync_write_param.length);

    if(dxl.syncWrite(sync_write_param)){
        ret = true;
    }

    return ret;
}

bool MecanumbotGrabber::control_grabber(int16_t left_grabber_value, int16_t right_grabber_value)
{
  bool dxl_comm_result = false;

  left_grabber_value  = constrain(left_grabber_value, LIMIT_GRABBER_MIN, LIMIT_GRABBER_MAX);
  right_grabber_value = constrain(right_grabber_value, LIMIT_GRABBER_MIN, LIMIT_GRABBER_MAX);

  dxl_comm_result = write_position((int16_t)left_grabber_value, (int16_t)right_grabber_value);
  if (dxl_comm_result == false)
    return false;

  return true;
}