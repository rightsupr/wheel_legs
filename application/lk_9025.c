#include "lk_9025.h"
#include "can_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "user_lib.h"
#include <math.h>

#define LK_TORQUE_CONSTANT 0.32f
#define LK_CURRENT_2_DATA 62.5f
#define PI  3.14159265358979f
extern CAN_HandleTypeDef hcan1;  //声明外部变量
extern CAN_HandleTypeDef hcan2;

static unsigned char is_init = 0;   //静态变量使得无法被外部声明，只在当前文件有效
static uint8_t motors_len = 0;
static struct Lk9025 *motors[3];
extern struct Lk9025 wheel[3];
static CAN_TxHeaderTypeDef
    tx_msg = {0x00, 0, CAN_ID_STD, CAN_RTR_DATA, 0x08, DISABLE};
static CAN_RxHeaderTypeDef rx_msg;

static uint8_t rx_data[9];

static void lk9025_register(struct Lk9025 *motor) {
  motors[motors_len] = motor;
  ++motors_len;
}                //注册电机，这本质与定义变量并再给变量赋值是一个意思（如下的初始化过程）

void lk9025_init(struct Lk9025 *motor, uint32_t device_id) {
  motor->id = device_id;

  motor->torque = 0;
  motor->angular_vel = 0;

  motor->last_heartbeat_timestamp_ms = 0;

  lk9025_register(motor);
}

void lk9025_set_enable(enum CanType can_type, enum Lk9025SendID CMD_ID) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0x88;
  tx_data[1] = 0;
  tx_data[2] = 0;
  tx_data[3] = 0;
  tx_data[4] = 0;
  tx_data[5] = 0;
  tx_data[6] = 0;
  tx_data[7] = 0;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void lk9025_torque_set(enum CanType can_type, enum Lk9025SendID CMD_ID, float motor_torque) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  int16_t motor_data;


  motor_data = motor_torque*199.3;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0xA1;
  tx_data[1] = 0;
  tx_data[2] = 0;
  tx_data[3] = 0;
  tx_data[4] = *(uint8_t *) (&motor_data);
  tx_data[5] = *((uint8_t *) (&motor_data) + 1);
//  tx_data[4] = motor_data << 8;
//  tx_data[5] = motor_data >> 8;
  tx_data[6] = 0;
  tx_data[7] = 0;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void lk9025_position_set(enum CanType can_type, enum Lk9025SendID CMD_ID, float position, uint16_t maxSpeed, uint8_t spinDirection) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  int32_t motor_data;
  motor_data = position*100;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0xA6;
  tx_data[1] = spinDirection;
  tx_data[2] = *(uint8_t *) (&maxSpeed);
  tx_data[3] = *((uint8_t *) (&maxSpeed) + 1);
  tx_data[4] = *(uint8_t *) (&motor_data);
  tx_data[5] = *((uint8_t *) (&motor_data) + 1);
  tx_data[6] = *((uint8_t *) (&motor_data) + 2);
  tx_data[7] = *((uint8_t *) (&motor_data) + 3);

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void lk9025_power(enum CanType can_type, enum Lk9025SendID CMD_ID,float power) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  int32_t motor_data;
  motor_data = power*199.3;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0xA0;
  tx_data[1] = 0x00;
  tx_data[2] = 0x00;
  tx_data[3] = 0x00;
  tx_data[4] = *(uint8_t *) (&motor_data);
  tx_data[5] = *((uint8_t *) (&motor_data) + 1);
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void lk9025_multi_torque_set(enum CanType can_type, float motor1_torque, float motor2_torque) {
  tx_msg.StdId = 0x280;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  float motor1_current, motor2_current;
  int16_t motor1_data, motor2_data;
  motor1_current = motor1_torque / LK_TORQUE_CONSTANT;
  motor2_current = motor2_torque / LK_TORQUE_CONSTANT;

  motor1_data = motor1_current * LK_CURRENT_2_DATA;
  motor2_data = motor2_current * LK_CURRENT_2_DATA;

  uint8_t tx_data[8] = {0};
//  tx_data[0] = *(uint8_t *) (&motor1_data);
//  tx_data[1] = *((uint8_t *) (&motor1_data) + 1);
//  tx_data[2] = *(uint8_t *) (&motor2_data);
//  tx_data[3] = *((uint8_t *) (&motor2_data) + 1);
  tx_data[0] = motor1_data << 8;
  tx_data[1] = motor1_data >> 8;
  tx_data[2] = motor2_data << 8;
  tx_data[3] = motor2_data >> 8;
  tx_data[4] = 0;
  tx_data[5] = 0;
  tx_data[6] = 0;
  tx_data[7] = 0;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
} //enum 枚举类型   can1_send_mail_box 用于判断邮箱信息有没有成功被发出去

void lk9025_clear_motor_errors(enum CanType can_type, enum Lk9025SendID CMD_ID) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0x9B;
  tx_data[1] = 0;
  tx_data[2] = 0;
  tx_data[3] = 0;
  tx_data[4] = 0;
  tx_data[5] = 0;
  tx_data[6] = 0;
  tx_data[7] = 0;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}   

void lk9025_encoder_read(enum CanType can_type, enum Lk9025SendID CMD_ID) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0x94;
  tx_data[1] = 0x00;
  tx_data[2] = 0x00;
  tx_data[3] = 0x00;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void lk9025_encoder_read_back(uint32_t id, uint8_t data[]) {
  uint32_t encoder_int;
  switch (id) {
    case HEAD_RECEIVE: {
      encoder_int = (uint32_t)((data)[7] << 24 | (data)[6] << 16 | (data)[5] << 8 | (data)[4]);
      // 使用 fmod 截取360度范围
      wheel[2].encoder = encoder_int/100.0;
      break;
    }
    default: break;
  }
}

void lk9025_can_msg_unpack(uint32_t id, uint8_t data[]) {
  int16_t speed_int, iq_int,encoder_int;
  switch (id) {
    case WHEEL_L_RECEIVE: {
      speed_int = (int16_t) ((data)[5] << 8 | (data)[4]);
      iq_int = (int16_t) ((data)[3] << 8 | (data)[2]);
      encoder_int = (int16_t) ((data)[7] << 8 | (data)[6]);
      motors[0]->angular_vel = speed_int * PI / 180;
      wheel[0].angular_vel = speed_int * PI / 180;
      motors[0]->torque = iq_int/199.3;
      wheel[0].torque = iq_int/199.3;
      motors[0]->encoder = encoder_int/1000.0;
      wheel[0].encoder = encoder_int/1000.0;
      break;
    }

    case WHEEL_R_RECEIVE: {
      speed_int = (int16_t) ((data)[5] << 8 | (data)[4]);
      iq_int = (int16_t) ((data)[3] << 8 | (data)[2]);
      encoder_int = (int16_t) ((data)[7] << 8 | (data)[6]);
      motors[1]->angular_vel = speed_int * PI / 180;
      wheel[1].angular_vel = speed_int * PI / 180;
      motors[1]->torque = iq_int/199.3;
      wheel[1].torque = iq_int/199.3;
      motors[1]->encoder = encoder_int/1000.0;
      wheel[1].encoder = encoder_int/1000.0;
      break;
    }

    case HEAD_RECEIVE: {
      
      encoder_int = (int16_t) ((data)[7] << 8 | (data)[6]);
      motors[2]->encoder = encoder_int/182.04;
      wheel[2].encoder = encoder_int/182.04;
      break;
    }
    default:break;
  }
}  //把获得的数据更新到了motors这个数组里

