#ifndef CAN_DEVICE_H
#define CAN_DEVICE_H

#include "stdint.h"

enum CanType {
  CAN_1,
  CAN_2,
};

enum lk8010SendID {
  JOINT_LF_SEND = 0x143,
  JOINT_LB_SEND = 0x144,
  JOINT_RF_SEND = 0x141,
  JOINT_RB_SEND = 0x142,
};

enum lk8016SendID {
  JOINT_PF_SEND = 0x143,
  JOINT_PB_SEND = 0x144,
};
enum Lk9025SendID {
  WHEEL_L_SEND = 0x141,
  WHEEL_R_SEND = 0x142,
  HEAD_SEND=0x145,
};

enum GimbalSendID {
  CHASSIS_ANGLE_VEL_INFO = 0x111,
};
//L是膝关节，R是髋关节，P是肩关节
//  JOINT_LB_RECEIVE = 0x144,JOINT_RF_RECEIVE = 0x141,右 
//  JOINT_LF_RECEIVE = 0x143,JOINT_RB_RECEIVE = 0x142,左
enum CanReceiveDeviceId {
  WHEEL_L_RECEIVE = 0x141,
  WHEEL_R_RECEIVE = 0x142,
  JOINT_LF_RECEIVE = 0x143,
  JOINT_LB_RECEIVE = 0x144,
  JOINT_RF_RECEIVE = 0x141,
  JOINT_RB_RECEIVE = 0x142,
	JOINT_PF_RECEIVE = 0x143,
  JOINT_PB_RECEIVE = 0x144,
  HEAD_RECEIVE=0x145,
  CHASSIS_MODE_HEIGHT_INFO = 0x101,
  CHASSIS_SPEED_INFO = 0x102,
  CHASSIS_ANGLE_INFO = 0x103,
};   //定义了收信息的id

void can_filter_init(void);

uint32_t get_can1_free_mailbox();
uint32_t get_can2_free_mailbox();

#endif //CAN_DEVICE_H
