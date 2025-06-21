
#include "red_led_task.h"
//#include "usbd_cdc_if.h"                  // Device header
#include "INS_task.h"                  // Device header

#include "cmsis_os.h"
#include "main.h"

extern void usart_printf(const char *fmt,...);
extern fp32 INS_angle[3];
/**
  * @brief          red led task
  * @param[in]      argument: NULL
  * @retval         none
  */
/**
  * @brief          ºìµÆÈÎÎñ
  * @param[in]      argument: NULL
  * @retval         none
  */
void red_led_task(void const * argument)
{

    while(1)
    {
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        osDelay(200);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        osDelay(200);
				//uint8_t data[] = "Hello, USB!";
				//uint16_t len = sizeof(data);
				//CDC_Transmit_FS(data, len);
				
    }
}


