#include "lk_8010.h"
#include "can_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "user_lib.h"
#include "chassis.h"
#define PI  3.14159265358979f
extern CAN_HandleTypeDef hcan1;  //声明外部变量
extern CAN_HandleTypeDef hcan2;


static unsigned char is_init = 0;   //静态变量使得无法被外部声明，只在当前文件有效
static uint8_t motors_len = 0;
static struct Lk8010 *motors[4];
extern struct Lk8010 joint[4];
static CAN_TxHeaderTypeDef
    tx_msg = {0x00, 0, CAN_ID_STD, CAN_RTR_DATA, 0x08, DISABLE};
static CAN_RxHeaderTypeDef rx_msg;

static uint8_t rx_data[9];

static void lk8010_register(struct Lk8010 *motor) {
  motors[motors_len] = motor;
  ++motors_len;
}                //注册电机，这本质与定义变量并再给变量赋值是一个意思（如下的初始化过程）

void lk8010_init(struct Lk8010 *motor, uint32_t device_id) {
  motor->id = device_id;

  motor->torque = 0;
  motor->angular_vel = 0;

  motor->last_heartbeat_timestamp_ms = 0;

  lk8010_register(motor);
}

void lk8010_set_enable(enum CanType can_type, enum lk8010SendID CMD_ID) {
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

void lk8010_torque_set(enum CanType can_type, enum lk8010SendID CMD_ID, float motor_torque) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  int16_t motor_data;


  motor_data = motor_torque*16.535;

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

void lk8010_position_set(enum CanType can_type, enum lk8010SendID CMD_ID, float position, uint16_t maxSpeed, uint8_t spinDirection) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  int32_t motor_data;
  motor_data = position*36*100;

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

void lk8010_encoder_read(enum CanType can_type, enum lk8010SendID CMD_ID) {
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

void lk8010_clear_motor_errors(enum CanType can_type, enum lk8010SendID CMD_ID) {
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

void lk8010_encoder_read_back(uint32_t id, uint8_t data[]) {
  uint32_t encoder_int;
  switch (id) {
    case JOINT_LF_RECEIVE: {
      encoder_int = (uint32_t)((data)[7] << 24 |(data)[6] << 16 | (data)[5] << 8 | (data)[4]);
      joint[0].encoder = encoder_int/3600.0;
      break;
    }

    case JOINT_LB_RECEIVE: {
      encoder_int = (uint32_t)((data)[7] << 24 |(data)[6] << 16 | (data)[5] << 8 | (data)[4]);
      joint[1].encoder = encoder_int/3600.0;
      break;
    }
    case JOINT_RF_RECEIVE: {
      encoder_int = (uint32_t)((data)[7] << 24 |(data)[6] << 16 | (data)[5] << 8 | (data)[4]);
      joint[2].encoder = encoder_int/3600.0;
      break;
    }

    case JOINT_RB_RECEIVE: {
      encoder_int = (uint32_t)((data)[7] << 24 |(data)[6] << 16 | (data)[5] << 8 | (data)[4]);
      joint[3].encoder = encoder_int/3600.0;
      break;
    }
    default:break;
  }

}

void lk8010_can_msg_unpack(uint32_t id, uint8_t data[]) {
  int16_t speed_int, iq_int;
  uint16_t encoder_int;
  switch (id) {
    case JOINT_LF_RECEIVE: {
      speed_int = (int16_t) ((data)[5] << 8 | (data)[4]);
      iq_int = (int16_t) ((data)[3] << 8 | (data)[2]);
      encoder_int = (uint16_t) ((data)[7] << 8 | (data)[6]);
      motors[0]->angular_vel = speed_int * PI / 180;
      joint[0].angular_vel = speed_int * PI / 180;
      motors[0]->torque = iq_int/16.535;
      joint[0].torque = iq_int/16.535;
      motors[0]->encoder = encoder_int/182.04;
      joint[0].encoder = encoder_int/182.04;
      break;
    }

    case JOINT_LB_RECEIVE: {
      speed_int = (int16_t) ((data)[5] << 8 | (data)[4]);
      iq_int = (int16_t) ((data)[3] << 8 | (data)[2]);
      encoder_int = (uint16_t) ((data)[7] << 8 | (data)[6]);
      motors[1]->angular_vel = speed_int * PI / 180;
      joint[1].angular_vel = speed_int * PI / 180;
      motors[1]->torque = iq_int/16.535;
      joint[1].torque = iq_int/16.535;
      motors[1]->encoder = encoder_int/182.04;
      joint[1].encoder = encoder_int/182.04;
      break;
    }

    case JOINT_RF_RECEIVE: {
      speed_int = (int16_t) ((data)[5] << 8 | (data)[4]);
      iq_int = (int16_t) ((data)[3] << 8 | (data)[2]);
      encoder_int = (uint16_t) ((data)[7] << 8 | (data)[6]);
      motors[2]->angular_vel = speed_int * PI / 180;
      joint[2].angular_vel = speed_int * PI / 180;
      motors[2]->torque = iq_int/16.535;
      joint[2].torque = iq_int/16.535;
      motors[2]->encoder = encoder_int/182.04;
      joint[2].encoder = encoder_int/182.04;
      break;
    }

    case JOINT_RB_RECEIVE: {
      speed_int = (int16_t) ((data)[5] << 8 | (data)[4]);
      iq_int = (int16_t) ((data)[3] << 8 | (data)[2]);
      encoder_int = (uint16_t) ((data)[7] << 8 | (data)[6]);
      motors[3]->angular_vel = speed_int * PI / 180;
      joint[3].angular_vel = speed_int * PI / 180;
      motors[3]->torque = iq_int/16.535;
      joint[3].torque = iq_int/16.535;
      motors[3]->encoder = encoder_int/182.04;
      joint[3].encoder = encoder_int/182.04;
      break;
    }
    default:break;
    
  }
}  //把获得的数据更新到了motors这个数组里

