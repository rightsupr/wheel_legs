#include "Calibrate.h"
  #include "string.h"
  #include "cmsis_os.h"
  #include "main.h"
  //#include "bsp_adc.h"
  #include "buzzer.h"
  #include "flash.h"
  #include "red_led_task.h"
  #include "ins_task.h"
  
  //include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
  #define FLASH_WRITE_BUF_LENGHT  (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGHT * 4)
  
  
  /**
    * @brief          从flash读取校准数据
    * @param[in]      none
    * @retval         none
    */
  static void cali_data_read(void);
  
  /**
    * @brief          往flash写入校准数据
    * @param[in]      none
    * @retval         none
    */
  static void cali_data_write(void);
  
  /**
    * @brief          "head"设备校准
    * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
    * @param[in]      cmd:
                      CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                      CALI_FUNC_CMD_ON: 代表需要校准
    * @retval         0:校准任务还没有完
                      1:校准任务已经完成
    */
  static unsigned char cali_head_hook(uint32_t *cali, unsigned char cmd);   //header device cali function
  
  /**
    * @brief          陀螺仪设备校准
    * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
    * @param[in]      cmd:
                      CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                      CALI_FUNC_CMD_ON: 代表需要校准
    * @retval         0:校准任务还没有完
                      1:校准任务已经完成
    */
  static unsigned char cali_gyro_hook(uint32_t *cali, unsigned char cmd);   //gyro device cali function
  
  /**
    * @brief          云台设备校准
    * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
    * @param[in]      cmd:
                      CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                      CALI_FUNC_CMD_ON: 代表需要校准
    * @retval         0:校准任务还没有完
                      1:校准任务已经完成
    */
  static unsigned char cali_gimbal_hook(uint32_t *cali, unsigned char cmd); //gimbal device cali function
  
  
  
  #if INCLUDE_uxTaskGetStackHighWaterMark
  uint32_t calibrate_task_stack;
  #endif

  head_cali_t     head_cali;       //head cali data
  gimbal_cali_t   gimbal_cali;     //gimbal cali data
  imu_cali_t      accel_cali;      //accel cali data
  imu_cali_t      gyro_cali;       //gyro cali data
  imu_cali_t      mag_cali;        //mag cali data
  
  uint8_t CALI_FUNC_CMD=CALI_FUNC_CMD_ON;
  static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];
  uint16_t notify_flag=0;
  cali_sensor_t cali_sensor[CALI_LIST_LENGHT];
  
  static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"};
  
  TaskHandle_t Cail_task_local_handler;
  //cali data address
  static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
          (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
          (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
          (uint32_t *)&mag_cali
  };
  
  
  static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
          {
                  sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
                  sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};
  
  void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};
  
  static uint32_t calibrate_systemTick;
  
  /**
   * @brief 校准任务函数
   * 
   * 该函数是一个任务函数，用于执行传感器校准操作。它会不断循环，检查并执行各个传感器的校准命令。
   * 
   * @param pvParameters 任务参数，未使用
   * 
   * 任务流程：
   * 1. 调用 RC_cmd_to_calibrate() 函数，获取校准命令。
   * 2. 遍历所有传感器，检查是否有校准命令需要执行。
   * 3. 如果传感器有校准命令且校准钩子函数不为空，则调用校准钩子函数。
   * 4. 如果校准钩子函数返回成功，则设置传感器名称和校准完成标志，并清除校准命令。
   * 5. 调用 cali_data_write() 函数，将校准数据写入存储。
   * 6. 调用 osDelay() 函数，延迟一段时间后继续下一次循环。
   * 7. 如果启用了 INCLUDE_uxTaskGetStackHighWaterMark 宏，则获取任务堆栈的高水位标记。
   * 
   * @note 该函数是一个无限循环任务，必须在 FreeRTOS 任务调度器中运行。
   */
  void calibrate_task(void const *pvParameters)
  {
    cali_param_init();  
    Cail_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    static uint8_t buzzer_executed = 0;  // 新增标记变量
    while (1)
    {
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
        if (!buzzer_executed)
        {
            buzzer_on(4, 10000); 
            osDelay(70);
            buzzer_off();
            buzzer_executed = 1;  // 标记已执行
        }
        if (cali_sensor[CALI_GYRO].cali_hook != NULL)
        {
						//cali_sensor_buf[CALI_GYRO]对应的是gyro_cali的地址（这又是最上面定义的偏移和比例的数据结构体）
            //在cali_param_init的时候把cali_sensor[i].flash_buf赋值为cali_sensor_buf[i]，所以这里的cali_sensor_buf[i]就是cali_sensor[i].flash_buf
            if (cali_sensor[CALI_GYRO].cali_hook(cali_sensor_buf[CALI_GYRO], CALI_FUNC_CMD))
            {
                // 校准完成，设置传感器名称和校准完成标志
                //HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
                //osDelay(200);
                //HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
                //osDelay(200);
                //当四万次迭代后才会进入此
                
                cali_sensor[CALI_GYRO].name[0] = cali_name[CALI_GYRO][0];
                cali_sensor[CALI_GYRO].name[1] = cali_name[CALI_GYRO][1];
                cali_sensor[CALI_GYRO].name[2] = cali_name[CALI_GYRO][2];
                cali_sensor[CALI_GYRO].cali_done = CALIED_FLAG;

                // 清除校准命令
                cali_sensor[CALI_GYRO].cali_cmd = 0;

                // 将校准数据写入存储 照道理是已经实现了把数据写入flash，那么在ins_task.c中就可以读取到这些数据了，不必多次校准
                cali_data_write();
                
            }
        }

  #if INCLUDE_uxTaskGetStackHighWaterMark
          // 获取任务堆栈的高水位标记
			calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
  #endif
      }
  }
  
  /**
    * @brief          获取imu控制温度, 单位℃
    * @param[in]      none
    * @retval         imu控制温度
    */
  int8_t get_control_temperature(void)
  {
  
      return head_cali.temperature;
  }
  
  /**
    * @brief          获取纬度,默认22.0f
    * @param[out]     latitude:fp32指针
    * @retval         none
    */
  void get_flash_latitude(float *latitude)
  {
  
      if (latitude == NULL)
      {
  
          return;
      }
      if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
      {
          *latitude = head_cali.latitude;
      }
      else
      {
          *latitude = 22.0f;
      }
  }
  
  /**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
void cali_param_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (unsigned char(*)(uint32_t *, unsigned char))cali_hook_fun[i];
    }

    cali_data_read();

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
            if (cali_sensor[i].cali_hook != NULL)
            {
                //if has been calibrated, set to init
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
}

  /**
    * @brief          从flash读取校准数据
    * @param[in]      none
    * @retval         none
    */
  static void cali_data_read(void)
  {
      uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
      uint8_t i = 0;
      uint16_t offset = 0;
      for (i = 0; i < CALI_LIST_LENGHT; i++)
      {
  
          //read the data in flash,
          cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
  
          offset += cali_sensor[i].flash_len * 4;
  
          //read the name and cali leg_flag,
          cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);
  
          cali_sensor[i].name[0] = flash_read_buf[0];
          cali_sensor[i].name[1] = flash_read_buf[1];
          cali_sensor[i].name[2] = flash_read_buf[2];
          cali_sensor[i].cali_done = flash_read_buf[3];
  
          offset += CALI_SENSOR_HEAD_LEGHT * 4;
  
          if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
          {
              cali_sensor[i].cali_cmd = 1;
          }
      }
  }
  
  /**
    * @brief          往flash写入校准数据
    * @param[in]      none
    * @retval         none
    */
  static void cali_data_write(void)
  {
      uint8_t i = 0;
      uint16_t offset = 0;
  
  
      for (i = 0; i < CALI_LIST_LENGHT; i++)
      {
          //copy the data of device calibration data
          memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
          offset += cali_sensor[i].flash_len * 4;
  
          //copy the name and "CALI_FLAG" of device
          memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
          offset += CALI_SENSOR_HEAD_LEGHT * 4;
      }
  
      //erase the page
      cali_flash_erase(FLASH_USER_ADDR,1);
      //write data
      cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
  }
  
  /**
    * @brief          "head"设备校准
    * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
    * @param[in]      cmd:
                      CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                      CALI_FUNC_CMD_ON: 代表需要校准
    * @retval         0:校准任务还没有完
                      1:校准任务已经完成
    */
  static unsigned char cali_head_hook(uint32_t *cali, unsigned char cmd)
  {
      head_cali_t *local_cali_t = (head_cali_t *)cali;
      if (cmd == CALI_FUNC_CMD_INIT)
      {
  //        memcpy(&head_cali, local_cali_t, sizeof(head_cali_t));
  
          return 1;
      }
      // self id
      local_cali_t->self_id = SELF_ID;
      //imu control temperature
      local_cali_t->temperature = 35 + 10;
      //head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
      if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
      {
          local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
      }
  
      local_cali_t->firmware_version = FIRMWARE_VERSION;
      //shenzhen latitude
      local_cali_t->latitude = 22.0f;
  
      return 1;
  }
  
  /**
    * @brief          陀螺仪设备校准
    * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
    * @param[in]      cmd:
                      CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                      CALI_FUNC_CMD_ON: 代表需要校准
    * @retval         0:校准任务还没有完
                      1:校准任务已经完成
    */
  static unsigned char cali_gyro_hook(uint32_t *cali, unsigned char cmd)
  {
      imu_cali_t *local_cali_t = (imu_cali_t *)cali;
      if (cmd == CALI_FUNC_CMD_INIT)
      {
          gyro_set_cali(local_cali_t->scale, local_cali_t->offset);  
          return 0;
      }
      else if (cmd == CALI_FUNC_CMD_ON)
      {
          static uint16_t count_time = 0;
          static uint8_t buzzer_executed = 0;   // 新增标志，防止代码重复执行
          gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
          if (count_time > GYRO_CALIBRATE_TIME)
          {
              count_time = 0;
              CALI_FUNC_CMD = CALI_FUNC_CMD_INIT;
              if (!buzzer_executed)
              { 
                buzzer_on(4, 10000); osDelay(70);
                buzzer_off();       osDelay(70);
                buzzer_on(4, 10000); osDelay(70);
                buzzer_off();
                buzzer_executed = 1;
              }
              return 1;
          }
          else{
              notify_flag++;
              return 0;
          }
      }  
      return 0;
  }
  
  /**
    * @brief          云台设备校准
    * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
    * @param[in]      cmd:
                      CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                      CALI_FUNC_CMD_ON: 代表需要校准
    * @retval         0:校准任务还没有完
                      1:校准任务已经完成
    */
  static unsigned char cali_gimbal_hook(uint32_t *cali, unsigned char cmd)
  {
  
      gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
      if (cmd == CALI_FUNC_CMD_INIT)
      {
  //        set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
  //                             local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
  //                             local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);
  
          return 0;
      }
      else if (cmd == CALI_FUNC_CMD_ON)
      {
  //        if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
  //                                 &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
  //                                 &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle))
  //        {
              cali_buzzer_off();
  
              return 1;
  //        }
  //        else
          {
              gimbal_start_buzzer();
  
              return 0;
          }
      }
  
      return 0;
  }
