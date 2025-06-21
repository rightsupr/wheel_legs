/**
 * @brief GPIO中断回调函数
 * @param pin 中断线对应的GPIO引脚
 * @return void
 * @details
 * 该函数用于处理GPIO中断事件。根据不同的中断线，执行相应的处理逻辑。
 * 以下是具体的实现细节：
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_task.h"
             // Device header

#include "main.h"

#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
// INS_task.c
// 管理惯性测量单元的数据获取、处理及温度控制
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "ahrs.h"

 

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm control the temperature of bmi088, 0-1000

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have 
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          控制BMI088传感器的温度。
  * @param[in]      temp: BMI088传感器的当前温度。
  * @retval         无
  * 
  * 该函数使用PID控制器来调节BMI088传感器的温度。
  * 如果传感器处于初始加热阶段，它会应用最大功率直到温度达到45摄氏度。一旦温度稳定，它会使用PID控制器
  * 将温度维持在45摄氏度。
  * 
  * 该函数还包括一个安全机制，以确保PID输出不会低于零。
  * 
  * `first_temperate`标志用于指示初始加热阶段是否完成。
  * `temp_constant_time`计数器用于跟踪在初始加热阶段温度超过45摄氏度的持续时间。
  */
static void imu_temp_control(fp32 temp);
/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

extern SPI_HandleTypeDef hspi1;
extern void usart_printf(const char *fmt,...);

static TaskHandle_t INS_task_local_handler;
extern TaskHandle_t Cail_task_local_handler;
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];
fp32 temperate;
static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
pid_type_def imu_temp_pid;

static const float timing_time = 0.001f;   

static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.ŷ���� ��λ rad

/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

// IMU传感器读取任务，负责更新姿态估计和温度控制
void INS_task(void const *pvParameters)
{
    //wait a time
    osDelay(INS_TASK_INIT_TIME);  //挂起7ms，其他任务可以执行，7ms结束后，INS_task处于就绪状态，等待其他任务执行完毕或者释放cpu
    while(BMI088_init())
    {
        osDelay(100);
    }
    while(ist8310_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    //旋转并解决零漂，但ist8310_real_data其实在下面并没用到磁力计
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
//温度控制初始化(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    //四元数初始化
    AHRS_init(INS_quat, INS_accel, INS_mag);
    //低通滤波器初始化
    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
      //xTaskGetHandle()获得当前任务的句柄
  //pcTaskGetName(NULL)：这个函数返回当前任务的名字。当传入 NULL 时，返回当前执行任务的名字。任务名字在创建任务时会被指定，但如果没有指定，返回的名字会是 "NULL"。
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    //是在配置 SPI 接口的通信速率。SPI_BAUDRATEPRESCALER_8 => 波特率为主时钟频率的 1/8。
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    //spi初始化，再检测是否成功，失败则Error_Handler()处理错误
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
    
    while (1)
    {
        //任务通知机制:任务可以通过 xTaskNotifyGive 或 xTaskNotify 等 API 给其他任务发送通知。
        //当前任务在此处暂停，直到收到通知信号。在等待期间，该任务不会消耗 CPU 资源。
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
        //在dma通信的地方会重新设置这些标志位
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);//取反，避免对于同一个数据处理
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);

        }
        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }
        temperate=bmi088_real_data.temp;
        if (temperate >= 43.5f && temperate <= 46.5f)
        {
            // 注意：请确保 calibration_task_handler 已经正确初始化
            xTaskNotifyGive(Cail_task_local_handler);

        }
        //旋转和零漂
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
        //加速度低通    加速度数据的加权平均来实现滤波操作，降低噪声和抖动，并在下一次循环赋给滤波器2，再下次循环，滤波器2赋值给1
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];


        AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);  //���ڷ�װ�õĿ�����
				//usart_printf("%f,%f,%f,%f\n", INS_angle[0] * 57.3, INS_angle[1] * 57.3, INS_angle[2] * 57.3, bmi088_real_data.temp);
        //because no use ist8310 and save time, no use
        
        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1<< IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);
//            ist8310_read_mag(ist8310_real_data.mag);
        }

    }
}




/**
    * @brief          旋转陀螺仪、加速度计和磁力计，并计算零漂，因为传感器的安装方向不同。
    * @param[out]     gyro: 加上零漂和旋转后的陀螺仪数据
    * @param[out]     accel: 加上零漂和旋转后的加速度计数据
    * @param[out]     mag: 加上零漂和旋转后的磁力计数据
    * @param[in]      bmi088: 陀螺仪和加速度计数据
    * @param[in]      ist8310: 磁力计数据
    * @retval         无
    */

static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }  //gyro_offset来自于gyro_offset_calc()函数，用于计算陀螺仪的零漂
}

/**
    * @brief          控制BMI088传感器的温度。
    * @param[in]      temp: BMI088传感器的当前温度。
    * @retval         无
    * 
    * 该函数使用PID控制器来调节BMI088传感器的温度。
    * 如果传感器处于初始加热阶段，它会应用最大功率直到温度达到45摄氏度。一旦温度稳定，它会使用PID控制器
    * 将温度维持在45摄氏度。
    * 
    * 该函数还包括一个安全机制，以确保PID输出不会低于零。
    * 
    * `first_temperate`标志用于指示初始加热阶段是否完成。
    * `temp_constant_time`计数器用于跟踪在初始加热阶段温度超过45摄氏度的持续时间。
    * 
    * 详细实现逻辑:
    * 1. 如果处于初始加热阶段(first_temperate为真):
    *    - 使用PID控制器计算输出
    *    - 确保PID输出不小于0
    *    - 将PID输出转换为PWM信号控制温度
    * 2. 如果不在初始加热阶段:
    *    - 应用最大功率加热
    *    - 当温度超过45摄氏度且持续200次循环后，退出初始加热阶段
    *    - 初始化PID控制器状态以维持稳定温度
    */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;  //用来保存计算得到的 PWM 控制信号，表示加热或冷却的功率。
    static uint8_t temp_constant_time = 0; //用于记录温度持续大于目标温度的时间
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 45);   //后来45度会有get_control_temperature代替，是目标温度值
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);   //pwm控制
    }
    else
    {
        //当初始加热到一定状态，超过目标值200个周期，标志位置1，初始积分输出设置为最大功率/2，并进入第一个if
        if (temp > 45)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);  //初始加热状态，最大功率-1
    }
}

/**
  * @brief          计算陀螺仪的零漂
  * @param[out]     gyro_offset:零漂值
  * @param[in]      gyro:表示陀螺仪的原始数据
  * @param[out]     offset_time_count: +1 auto  可能用于记录累积计算零漂的次数。
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }
    //最重要的核心，当4w次迭代后，零漂值就会稳定下来，这个值就是零漂值，此时gyro的值就会是0附近的值
        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}


/**
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         无
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

/**
    * @brief          设置陀螺仪的校准参数
    * @param[in]      cali_scale: 比例，默认值为1.0
    * @param[in]      cali_offset: 零漂
    * @retval         无
    * @details        该函数用于设置陀螺仪的校准参数，包括比例和零漂。校准参数通常从flash中读取。
    *                 该函数会将传入的校准参数保存到全局变量中，以便在后续的陀螺仪数据处理中使用。
    * 如同校准的函数，这个就是把cali_sensor[i].flash_buf中的值通过不断赋值，最终在该函数中给到gyro_offset，即全局变量
    */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief          get the quat
  * @param[in]      none
  * @retval         the point of INS_quat
  */

const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}
/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */

const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */

extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}
/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_accel
  */

extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}
/**
  * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll-axis unit ut
  * @param[in]      none
  * @retval         the point of INS_mag
  */

extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}


/**
 * @brief GPIO中断回调函数
 * @param GPIO_Pin 中断线对应的GPIO引脚
 * @retval none
 * @details
 * 该函数用于处理GPIO中断事件。根据不同的中断线，执行相应的处理逻辑。
 * 具体实现包括：
 * 1. 加速度计和陀螺仪数据准备中断处理
 * 2. 磁力计数据准备中断处理
 * 3. 任务唤醒中断处理
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        // 加速度计数据准备中断处理
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        // 陀螺仪数据准备中断处理
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        // 磁力计数据准备中断处理
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        // 任务唤醒中断处理
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
  * @brief          根据 imu_update_flag 的状态，启动相应的 SPI DMA 传输。
  * @param[in]      none
  * @retval         none
  * @note           似乎没有磁力计spi dma传输
  */
static void imu_cmd_spi_dma(void)
{
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);  // 清除陀螺仪更新标志
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);  // 设置陀螺仪 SPI 传输标志
            // 通过 GPIO 控制片选信号
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            // 启动 SPI DMA 传输
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
            return;
        }
        // 加速度计操作同上
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }    
        // 温度传感器操作同上    
        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}

//spi dma interrupt
void DMA2_Stream2_IRQHandler(void)  
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
        //gyro read over
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);            
        }
        //accel read over
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }        
        imu_cmd_spi_dma();
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

// 提供外部访问的姿态角度指针
float* get_ins_angle() {
    return INS_angle;  
  }
// 返回陀螺仪数据指针
  float* get_ins_gyro() {
    return INS_gyro;
  }
// 返回加速度计数据指针
  float* get_ins_accel() {
    return INS_accel;
  }