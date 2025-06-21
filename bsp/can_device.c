#include "can_device.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "lk_9025.h"
#include "lk_8010.h"
#include "lk_8016.h"
#include "chassis.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern volatile bool uart_received;
void can_filter_init(void) {
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);  //使用 HAL_CAN_ConfigFilter 函数将 can_filter_st 的配置应用到 CAN1 控制器中。
  HAL_CAN_Start(&hcan1);    //启动 CAN1 控制器
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);   //激活 CAN1 的接收中断功能，当 CAN1 接收到符合过滤器条件的消息存入 FIFO 0 后，将触发 HAL_CAN_RxFifo0MsgPendingCallback 回调函数。

  can_filter_st.SlaveStartFilterBank = 14;    // can_filter_st.SlaveStartFilterBank = 14; 的意义在于告诉系统：CAN1 使用过滤器 0 到 13。CAN2 使用过滤器 14 到最高值。
  can_filter_st.FilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}     //在main初始化了，将can的滤波器启动了，确保两者能接收符合条件的消息。启动 CAN1 和 CAN2 控制器，进入正常工作状态。启用 CAN 接收中断，确保当有消息接收时触发回调函数。

uint32_t get_can1_free_mailbox() {
  if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET) {
    return CAN_TX_MAILBOX0;
  } else if ((hcan1.Instance->TSR & CAN_TSR_TME1)
      != RESET) { return CAN_TX_MAILBOX1; }
  else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET) { return CAN_TX_MAILBOX2; }
  else { return 0; }
}

uint32_t get_can2_free_mailbox() {
  if ((hcan2.Instance->TSR & CAN_TSR_TME0) != RESET) {
    return CAN_TX_MAILBOX0;
  } else if ((hcan2.Instance->TSR & CAN_TSR_TME1)
      != RESET) { return CAN_TX_MAILBOX1; }
  else if ((hcan2.Instance->TSR & CAN_TSR_TME2) != RESET) { return CAN_TX_MAILBOX2; }
  else { return 0; }
}    

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8]={0};

  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
      // 如果接收到的StdId为1或2，则认为是lk9025的数据
      if (rx_header.StdId == WHEEL_L_RECEIVE || rx_header.StdId == WHEEL_R_RECEIVE||rx_header.StdId == HEAD_RECEIVE) {
          if (!uart_received) {
          lk9025_encoder_read_back(rx_header.StdId,rx_data);
        } else {
          lk9025_can_msg_unpack(rx_header.StdId, rx_data);
        }
      }else {  // 否则认为是lk8010的数据，根据uart_received区分处理函数
          if (!uart_received) {
              lk8010_encoder_read_back(rx_header.StdId, rx_data);
          } else {
              lk8010_can_msg_unpack(rx_header.StdId, rx_data);
          }
      }
  }
  } else if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
      //dm8009_can_msg_unpack(rx_header.StdId, rx_data);
      if (rx_header.StdId == JOINT_RF_RECEIVE || rx_header.StdId == JOINT_RB_RECEIVE) {
        if (!uart_received) {
          lk8010_encoder_read_back(rx_header.StdId, rx_data);
      } else {
        lk8010_can_msg_unpack(rx_header.StdId, rx_data);
      }
    }else {  // 否则认为是lk8010的数据，根据uart_received区分处理函数
      
      if (!uart_received) {
        lk8016_encoder_read_back(rx_header.StdId, rx_data);
    } else {
      lk8016_can_msg_unpack(rx_header.StdId, rx_data);
    }
    } 
    }
  }
}    //与can的过滤器有关，在过滤器启动了中断，并通过lk9025_can_msg_unpack赋值到了motors