#ifndef WHEEL_LEG_APP_INC_BOT_DRIVER_LK_8010_H_
#define WHEEL_LEG_APP_INC_BOT_DRIVER_LK_8010_H_

#include "stdint.h"
#include "can_device.h"

struct Lk8010{
  uint32_t id;
  float angular_vel;
  float torque;
  float encoder;
  uint32_t last_heartbeat_timestamp_ms;
};

void lk8010_init(struct Lk8010 *motor, uint32_t device_id);

void lk8010_set_enable(enum CanType can_type, enum lk8010SendID CMD_ID);

void lk8010_torque_set(enum CanType can_type, enum lk8010SendID CMD_ID, float motor_torque);

void lk8010_position_set(enum CanType can_type, enum lk8010SendID CMD_ID, float position, uint16_t maxSpeed, uint8_t spinDirection);

void lk8010_clear_motor_errors(enum CanType can_type, enum lk8010SendID CMD_ID);

void lk8010_encoder_read(enum CanType can_type, enum lk8010SendID CMD_ID);

void lk8010_encoder_read_back(uint32_t id, uint8_t data[]);

void lk8010_can_msg_unpack(uint32_t id, uint8_t data[]);

#endif //WHEEL_LEG_APP_INC_BOT_DRIVER_LK_8010_H_
