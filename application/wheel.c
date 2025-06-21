#include "wheel.h"
#include "lk_9025.h"
#include "cmsis_os.h"
#include "lk_8010.h"
#include "lk_8016.h"

struct Lk9025 wheel[3];
struct Lk8010 joint[4];
struct Lk8016 shoulder[2];
void wheel_init() {
  lk9025_init(&wheel[0], WHEEL_L_RECEIVE);
  lk9025_init(&wheel[1], WHEEL_R_RECEIVE);
  lk9025_init(&wheel[2], HEAD_RECEIVE);
  lk9025_set_enable(CAN_1,WHEEL_L_SEND);
  lk9025_set_enable(CAN_1,WHEEL_R_SEND);
  lk9025_set_enable(CAN_1,HEAD_SEND);
}

void joint_init() {
  lk8010_init(&joint[0], JOINT_LF_RECEIVE);
  lk8010_init(&joint[1], JOINT_LB_RECEIVE);

  lk8010_init(&joint[2], JOINT_RF_RECEIVE);
  lk8010_init(&joint[3], JOINT_RB_RECEIVE);
	
	lk8016_init(&shoulder[0], JOINT_PF_RECEIVE);
  lk8016_init(&shoulder[1], JOINT_PB_RECEIVE);
  lk8010_set_enable(CAN_1,JOINT_LF_SEND);
  lk8010_set_enable(CAN_1,JOINT_LB_SEND);

  lk8010_set_enable(CAN_2,JOINT_RF_SEND);
  lk8010_set_enable(CAN_2,JOINT_RB_SEND);
	
	lk8016_set_enable(CAN_2,JOINT_PF_SEND);
  lk8016_set_enable(CAN_2,JOINT_PB_SEND);
}

void set_wheel_torque(float torque_nm_L, float torque_nm_R, float power) {
  lk9025_torque_set(CAN_1,WHEEL_L_SEND,torque_nm_L);
  osDelay(1);
  lk9025_torque_set(CAN_1,WHEEL_R_SEND,torque_nm_R);
  // osDelay(1);
  // lk9025_power(CAN_1,HEAD_SEND,power);
  // lk9025_torque_set(CAN_2,WHEEL_L_SEND,torque_nm_L);
  // osDelay(1);
  // lk9025_torque_set(CAN_2,WHEEL_R_SEND,torque_nm_R);
}

void set_wheel_position(float positions, uint16_t maxSpeed, uint8_t spinDirection){
  lk9025_position_set(CAN_1,HEAD_SEND,positions,maxSpeed,spinDirection);
}

void set_joint_torque(float torque_nm_L1, float torque_nm_L2) {
  // lk8010_torque_set(CAN_1,JOINT_LF_SEND,torque_nm_L1);
  // osDelay(1);
  // lk8010_torque_set(CAN_1,JOINT_LB_SEND,torque_nm_L2);
  // osDelay(1);
  // lk8010_torque_set(CAN_2,JOINT_RF_SEND,torque_nm_L1);
  // osDelay(1);
  // lk8010_torque_set(CAN_2,JOINT_RB_SEND,torque_nm_L2);
	// osDelay(1);
	lk8016_torque_set(CAN_2,JOINT_PF_SEND,torque_nm_L1);
  osDelay(1);
  lk8016_torque_set(CAN_2,JOINT_PB_SEND,torque_nm_L2);
}

void set_joint_position(float *positions, uint16_t maxSpeed, uint8_t *spinDirection) {
  lk8010_position_set(CAN_1,JOINT_LF_SEND,360-positions[1],maxSpeed,spinDirection[0]);
  osDelay(1);
  lk8010_position_set(CAN_1,JOINT_LB_SEND,positions[1],maxSpeed,spinDirection[1]);
  osDelay(1);
  lk8010_position_set(CAN_2,JOINT_RF_SEND,positions[2],maxSpeed,spinDirection[2]);
  osDelay(1);
  lk8010_position_set(CAN_2,JOINT_RB_SEND,360-positions[2],maxSpeed,spinDirection[3]);
  osDelay(1);
  lk8016_position_set(CAN_2,JOINT_PF_SEND,0,maxSpeed,spinDirection[4]);
  osDelay(1);
  lk8016_position_set(CAN_2,JOINT_PB_SEND,0,maxSpeed,spinDirection[5]);
  osDelay(1);
  lk9025_position_set(CAN_1,HEAD_SEND,0,maxSpeed,spinDirection[6]);  
}

struct Lk9025 *get_wheel_motors(){
  return wheel;
}