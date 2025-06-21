#ifndef WHEEL_H
#define WHEEL_H

#include "can_device.h"
#include "lk_9025.h"
#include "lk_8010.h"
#include "lk_8016.h"
void wheel_init();
void joint_init();
void set_wheel_torque(float torque_nm_L, float torque_nm_R, float power);
void set_wheel_position(float positions, uint16_t maxSpeed, uint8_t spinDirection);
void set_joint_torque(float torque_nm_L1, float torque_nm_L2);
void set_joint_position(float *positions, uint16_t maxSpeed, uint8_t *spinDirection);
struct Lk9025 *get_wheel_motors();

#endif //WHEEL_H
