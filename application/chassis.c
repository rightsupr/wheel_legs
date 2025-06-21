#include <stdbool.h>
#include <math.h>
#include "chassis.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "ins_task.h"
#include "user_lib.h"
#include "error.h"
#include "filter.h"
#include "INS_task.h"
#include "buzzer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "remote_control.h"
//#include "joint.h"


#define DEBUG 0
#define GRAVITY_A 9.8f



const struct ChassisPhysicalConfig chassis_physical_config = {0.075f,
                                                              13.250f,
                                                              0.32f,
                                                              0.2618f,
                                                              0.15f,
                                                              0.27f,
                                                              0.27f,
                                                              0.15f,
                                                              0.15f};

char UARTBUF[13]={0};
uint8_t len;
float arr[3]={0.0};
volatile bool uart_received = false;
uint8_t rc_received=0;
/*******************************************************************************
 *                                Remote control                               *
 *******************************************************************************/
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Z_CHANNEL 2
#define CHASSIS_PIT_CHANNEL 3
#define CHASSIS_ROLL_CHANNEL 0

#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)
#define RC_TO_PITCH ((MAX_PITCH-MIN_PITCH)/660)
#define RC_TO_ROLL ((MAX_ROLL-MIN_ROLL)/660)
#define RC_TO_L0  ((MAX_L0-MIN_L0)/1320)

/*******************************************************************************
 *                                PID parameters                               *
 *******************************************************************************/
#define CHASSIS_LEG_L0_PID_P 700
#define CHASSIS_LEG_L0_PID_I 0.02
#define CHASSIS_LEG_L0_PID_D 6000
#define CHASSIS_LEG_L0_PID_IOUT_LIMIT 10
#define CHASSIS_LEG_L0_PID_OUT_LIMIT 100000
//#define CHASSIS_LEG_L0_PID_OUT_LIMIT 0  腿部pid控制

#define CHASSIS_OFFGROUND_LEG_LO_PID_P 250
#define CHASSIS_OFFGROUND_LEG_L0_PID_I 0
#define CHASSIS_OFFGROUND_LEG_L0_PID_D 00000
#define CHASSIS_OFFGROUND_LEG_L0_PID_IOUT_LIMIT 3
#define CHASSIS_OFFGROUND_LEG_L0_PID_OUT_LIMIT 10000
//离地后的腿部运动
#define CHASSIS_VW_CURRENT_PID_P (-5)
#define CHASSIS_VW_CURRENT_PID_I (-0.1)
#define CHASSIS_VW_CURRENT_PID_D (-800)
#define CHASSIS_VW_CURRENT_PID_IOUT_LIMIT 0.1
#define CHASSIS_VW_CURRENT_PID_OUT_LIMIT 10
//猜测为轮子的力矩控制
//#define CHASSIS_VW_CURRENT_PID_P (-8.0)
//#define CHASSIS_VW_CURRENT_PID_I (-0.01)
//#define CHASSIS_VW_CURRENT_PID_D (-800)
//#define CHASSIS_VW_CURRENT_PID_IOUT_LIMIT 0.1
//#define CHASSIS_VW_CURRENT_PID_OUT_LIMIT 10

//#define CHASSIS_VW_CURRENT_PID_P 0.0
//#define CHASSIS_VW_CURRENT_PID_I 0
//#define CHASSIS_VW_CURRENT_PID_D -0
//#define CHASSIS_VW_CURRENT_PID_IOUT_LIMIT 0.2
//#define CHASSIS_VW_CURRENT_PID_OUT_LIMIT 0.5

#define CHASSIS_VW_SPEED_PID_P (-4.5)
#define CHASSIS_VW_SPEED_PID_I (-0.1)
#define CHASSIS_VW_SPEED_PID_D (20)
#define CHASSIS_VW_SPEED_PID_IOUT_LIMIT 0.1
#define CHASSIS_VW_SPEED_PID_OUT_LIMIT 3
//猜测是轮子的速度控制
#define CHASSIS_SPIN_PID_P (-0.15)
#define CHASSIS_SPIN_PID_I (-0.0)
#define CHASSIS_SPIN_PID_D (-0.5)
#define CHASSIS_SPIN_PID_IOUT_LIMIT 0.1
#define CHASSIS_SPIN_PID_OUT_LIMIT 3
//旋转控制
#define CHASSIS_ROLL_PID_P 0.7
#define CHASSIS_ROLL_PID_I 0.005
#define CHASSIS_ROLL_PID_D 0.1
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.01
#define CHASSIS_ROLL_PID_OUT_LIMIT 0.3
//#define CHASSIS_ROLL_PID_OUT_LIMIT 0 横滚控制

#define CHASSIS_LEG_COORDINATION_PID_P 100
#define CHASSIS_LEG_COORDINATION_PID_I 0.0
#define CHASSIS_LEG_COORDINATION_PID_D 300
#define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 2
#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 100
//#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 0  可能是多条腿的协调运动控制

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern fp32 temperate;
extern uint16_t notify_flag;
extern struct Lk9025 wheel[3];
extern struct Lk8010 joint[4];
extern struct Lk8016 shoulder[2];
extern void usart_printf(const char *fmt,...);
extern const RC_ctrl_t *local_rc_ctrl;
static uint8_t rc_sw_R_last;
static bool init_flag = false;
static bool is_joint_enable = false;
struct Chassis chassis;
struct MovingAverageFilter robot_az_filter;  //平均移动滤波器
pid_type_def wheel_pid;
pid_type_def speed_pid;
pid_type_def yaw_pid;
pid_type_def head_pid;
pid_type_def shoulder_pid;
static fp32 wheel_PID[3] = {-0.2f,0,-0.9};
static fp32 speed_PID[3] = {-0.15f, -0.00075f,00};
static fp32 yaw_PID[3] = {0.001, 0,0};
static fp32 head_PID[3] = {0.7,0.01,10};
static fp32 shoulder_PID[3] = {0,0,0};
float speed_target=0.0f;
float yaw_target=0.0f;
float pid_output=0.0f;
float speed_out=0.0;
float positions[7]={0.0};
uint16_t maxSpeed=720;
uint8_t spinDirection[7]={0};
float initial_yaw = 0.0f;
float initial_head_encoder=0.0f;
float yaw_encoder_error=0.0f;
float average_velocity = 0.0f;
float pitch_zero_offset=5.8;

float yaw_feedback= 0.0f; //偏航角反馈值

// float head_position=0.0;
// uint8_t head_spinDirection=0;
// static uint32_t last_update_time = 0; // 记录上一次调用的时间
float power=0.0;
float error_yaw=0.0;
/*******************************************************************************
 *                                    Init                                     *
 *******************************************************************************/
//设置底盘一系列的PID参数
static void chassis_pid_init() {
  pid_init(&chassis.chassis_vw_speed_pid,
           CHASSIS_VW_SPEED_PID_OUT_LIMIT,
           CHASSIS_VW_SPEED_PID_IOUT_LIMIT,
           CHASSIS_VW_SPEED_PID_P,
           CHASSIS_VW_SPEED_PID_I,
           CHASSIS_VW_SPEED_PID_D);

  pid_init(&chassis.chassis_vw_current_pid,
           CHASSIS_VW_CURRENT_PID_OUT_LIMIT,
           CHASSIS_VW_CURRENT_PID_IOUT_LIMIT,
           CHASSIS_VW_CURRENT_PID_P,
           CHASSIS_VW_CURRENT_PID_I,
           CHASSIS_VW_CURRENT_PID_D);

  pid_init(&chassis.chassis_spin_pid,
           CHASSIS_SPIN_PID_OUT_LIMIT,
           CHASSIS_SPIN_PID_IOUT_LIMIT,
           CHASSIS_SPIN_PID_P,
           CHASSIS_SPIN_PID_I,
           CHASSIS_SPIN_PID_D);

  pid_init(&chassis.leg_L.ground_pid,
           CHASSIS_LEG_L0_PID_OUT_LIMIT,
           CHASSIS_LEG_L0_PID_IOUT_LIMIT,
           CHASSIS_LEG_L0_PID_P,
           CHASSIS_LEG_L0_PID_I,
           CHASSIS_LEG_L0_PID_D);

  pid_init(&chassis.leg_R.ground_pid,
           CHASSIS_LEG_L0_PID_OUT_LIMIT,
           CHASSIS_LEG_L0_PID_IOUT_LIMIT,
           CHASSIS_LEG_L0_PID_P,
           CHASSIS_LEG_L0_PID_I,
           CHASSIS_LEG_L0_PID_D);

  pid_init(&chassis.chassis_roll_pid,
           CHASSIS_ROLL_PID_OUT_LIMIT,
           CHASSIS_ROLL_PID_IOUT_LIMIT,
           CHASSIS_ROLL_PID_P,
           CHASSIS_ROLL_PID_I,
           CHASSIS_ROLL_PID_D);

  pid_init(&chassis.chassis_leg_coordination_pid,
           CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
           CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
           CHASSIS_LEG_COORDINATION_PID_P,
           CHASSIS_LEG_COORDINATION_PID_I,
           CHASSIS_LEG_COORDINATION_PID_D);
}


void chassis_init() {
  osDelay(2000);
  chassis.leg_L.leg_index = L;  //就只是给底盘结构体赋值
  chassis.leg_R.leg_index = R;
  wheel_init();  //轮子初始化，给初始赋值，注册电机
  osDelay(2);
  joint_init();  //关节初始化
  chassis_pid_init(); //设置底盘一系列的PID参数
  chassis.chassis_ctrl_mode = CHASSIS_DISABLE;   //控制模式先为不使能
  chassis.is_chassis_offground = false;  //不离地
  init_filter(&robot_az_filter);   //滤波器初始化  这种带地址的，就是为了把值赋值到这个变量里

  chassis.chassis_ctrl_info.spin_speed = 5.0f;  //初始化旋转速度
  vTaskSuspendAll(); //挂起

  xTaskResumeAll();  //恢复
}
/*******************************************************************************
 *                                Getter&Setter                                *
 *******************************************************************************/
//获取IMU数据，并滤波（三轴角、角速度、角加速度，角加速度的滤波值、绕z轴的角速度）
static void get_IMU_info() {
  chassis.imu_reference.pitch_angle = -*(get_ins_angle() + 2);  //通过偏移访问不同轴的角度
  chassis.imu_reference.yaw_angle = -*(get_ins_angle() + 0);
  chassis.imu_reference.roll_angle = -*(get_ins_angle() + 1);
  //角速度
  chassis.imu_reference.pitch_gyro = -*(get_ins_gyro() + 0);
  chassis.imu_reference.yaw_gyro = -*(get_ins_gyro() + 2);
  chassis.imu_reference.roll_gyro = -*(get_ins_gyro() + 1);
  //角加速度
  chassis.imu_reference.ax = -*(get_ins_accel() + 1);
  chassis.imu_reference.ay = *(get_ins_accel() + 0);
  chassis.imu_reference.az = *(get_ins_accel() + 2);
  //重力补偿滤波
  chassis.imu_reference.ax_filtered = chassis.imu_reference.ax - GRAVITY_A * sinf(chassis.imu_reference.pitch_angle);
  chassis.imu_reference.ay_filtered = chassis.imu_reference.ay
      - GRAVITY_A * cosf(chassis.imu_reference.pitch_angle) *
          sinf(chassis.imu_reference.roll_angle);
  chassis.imu_reference.az_filtered = chassis.imu_reference.az
      - GRAVITY_A * cosf(chassis.imu_reference.pitch_angle) *
          cosf(chassis.imu_reference.roll_angle);
  float robot_az_raw = chassis.imu_reference.ax_filtered * sinf(chassis.imu_reference.pitch_angle)  
      + chassis.imu_reference.ay_filtered * sinf(-chassis.imu_reference.roll_angle)
          * cosf(chassis.imu_reference.pitch_angle)
      + chassis.imu_reference.az_filtered * cosf(chassis.imu_reference.pitch_angle)
          * cosf(chassis.imu_reference.roll_angle);  // Z 轴加速度计算

  updateFilter(&robot_az_filter, robot_az_raw);  //滤波
  chassis.imu_reference.robot_az = getFilteredValue(&robot_az_filter);  //获得绕z轴旋转的加速度
}


// static void set_chassis_mode() {
//   if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) {
//     chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//     chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
//     chassis.jump_state == NOT_READY;
//   } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && init_flag == false) {
//     chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//     chassis.chassis_ctrl_mode = CHASSIS_INIT;
//   } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && init_flag == true) {
//     chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//     chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
//     chassis.jump_state = NOT_READY;
//   } else if (switch_is_mid(rc_sw_R_last) && init_flag == true && switch_is_up(get_rc_ctrl()->rc.s[RC_s_R])) {
//     chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//     chassis.chassis_ctrl_mode = CHASSIS_JUMP;
//     chassis.jump_state = READY;
//   }
//   rc_sw_R_last = get_rc_ctrl()->rc.s[RC_s_R];
// }

// static void set_chassis_mode_from_gimbal_msg() {
//   if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_DISABLE) {
//     chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//     chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
//   } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_ENABLE && init_flag == false) {
//     chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//     chassis.chassis_ctrl_mode = CHASSIS_INIT;
//   } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_ENABLE && init_flag == true) {
//     chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//     chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
//   } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_SIDEWAYS) {
//     chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//     chassis.chassis_ctrl_mode = CHASSIS_SIDEWAYS;
//   }
// }

// static void chassis_device_offline_handle() {
//   check_is_rc_online(get_rc_ctrl());
//   if (get_errors() != 0) {
//     chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
//   }
// }
// //从遥控器控制器 (RC) 获取控制信息并将其更新到底盘的控制信息结构体 chassis_ctrl_info 
// static void set_chassis_ctrl_info() {
//   chassis.chassis_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;

//   chassis.chassis_ctrl_info.yaw_angle_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_Z_CHANNEL]) * -RC_TO_YAW_INCREMENT;
//   if (chassis.chassis_ctrl_info.yaw_angle_rad >= PI) {
//     chassis.chassis_ctrl_info.yaw_angle_rad -= 2 * PI;
//   } else if (chassis.chassis_ctrl_info.yaw_angle_rad <= -PI) {
//     chassis.chassis_ctrl_info.yaw_angle_rad += 2 * PI;
//   }

//   chassis.chassis_ctrl_info.pitch_angle_rad = (float) (get_rc_ctrl()->rc.ch[CHASSIS_PIT_CHANNEL]) * RC_TO_PITCH;

//   chassis.chassis_ctrl_info.height_m = -(float) (get_rc_ctrl()->rc.ch[4]) * RC_TO_L0 + DEFAULT_L0;
//   VAL_LIMIT(chassis.chassis_ctrl_info.height_m, MIN_L0, MAX_L0)
// }

// static void set_chassis_ctrl_info_from_gimbal_msg() {
//   chassis.chassis_ctrl_info.v_m_per_s = get_gimbal_msg()->chassis_ctrl_info.v_m_per_s;
//   chassis.chassis_ctrl_info.yaw_angle_rad = get_gimbal_msg()->chassis_ctrl_info.yaw_angle_rad;
//   chassis.chassis_ctrl_info.roll_angle_rad = get_gimbal_msg()->chassis_ctrl_info.roll_angle_rad;
//   chassis.chassis_ctrl_info.height_m = get_gimbal_msg()->chassis_ctrl_info.height_m;
// }

//讲计算出来的扭矩值发送给电机
static void chassis_motor_cmd_send() {
#if DEBUG
  set_joint_torque(0, 0, 0, 0);
  osDelay(2);
  set_wheel_torque(0, 0);

#else
  // set_joint_torque(-chassis.leg_L.joint_F_torque,
  //                  -chassis.leg_L.joint_B_torque,
  //                  chassis.leg_R.joint_F_torque,
  //                  chassis.leg_R.joint_B_torque);

//  set_joint_torque(0, 0, 0, 0);
  // buzzer_on(1000, 1000);
  //osDelay(2);

  spinDirection[0] = determine_spin_direction(360-positions[1], joint[0].encoder);
  spinDirection[1] = determine_spin_direction(positions[1], joint[1].encoder);
  spinDirection[2] = determine_spin_direction(positions[2], joint[2].encoder);
  spinDirection[3] = determine_spin_direction(360-positions[2], joint[3].encoder);
  spinDirection[4] = determine_spin_direction(0, shoulder[0].encoder);
  spinDirection[5] = determine_spin_direction(0, shoulder[1].encoder);
  spinDirection[6] = determine_spin_direction(0, wheel[2].encoder);
  // if (HAL_GetTick() - last_update_time > 50) {
  //   last_update_time = HAL_GetTick(); // 更新上一次调用的时间
  //   head_position = chassis.imu_reference.yaw_angle * 57.3;

  //   // 将 head_position 从 -180到180 转换为 0-360
  //   if (head_position < 0) {
  //       head_position += 360.0f;
  //   }
  //   head_spinDirection = determine_spin_direction(head_position, wheel[2].encoder);
  //   float wheel_error = fabs(head_position - wheel[2].encoder);
  //   if (wheel_error > 0.5f) { 
  //   set_wheel_position(head_position,maxSpeed,head_spinDirection);}
  // }else{
  //   lk9025_encoder_read(CAN_1, HEAD_RECEIVE);
  // }

  
  //set_joint_torque(-chassis.leg_L.joint_F_torque, -chassis.leg_L.joint_B_torque);
  if (rc_received==2||rc_received==3) {
      set_joint_position(positions, maxSpeed, spinDirection);
      set_wheel_torque(-chassis.leg_L.wheel_torque, chassis.leg_R.wheel_torque,-power);
      // set_joint_torque(chassis.leg_L.joint_F_torque,chassis.leg_L.joint_B_torque);
}
  //buzzer_off();
//  set_wheel_torque(0, 0);

#endif
}
//卡尔曼滤波计算，并将值赋给底盘结构体（未仔细看）


struct Chassis *get_chassis() {
  return &chassis;
}
/*******************************************************************************
 *                                    Task                                     *
 *******************************************************************************/
// static void chassis_init_task() {
//   chassis.chassis_ctrl_info.yaw_angle_rad = chassis.imu_reference.yaw_angle;
//   if (!is_joint_enable) {
//     set_dm8009_enable(CAN_2, JOINT_LF_SEND);
//     set_dm8009_enable(CAN_2, JOINT_LB_SEND);
//     HAL_Delay(2);
//     set_dm8009_enable(CAN_2, JOINT_RF_SEND);
//     set_dm8009_enable(CAN_2, JOINT_RB_SEND);
//     HAL_Delay(2);
//     is_joint_enable = true;
//     return;
//   }
//   if (is_joints_reduced()) {
//     init_flag = true;
//   } else {
//     set_dm8009_MIT(CAN_2, JOINT_LF_SEND, 0, 0, 20, 5, 0);
//     set_dm8009_MIT(CAN_2, JOINT_LB_SEND, 0, 0, 20, 5, 0);
//     HAL_Delay(2);
//     set_dm8009_MIT(CAN_2, JOINT_RF_SEND, 0, 0, 20, 5, 0);
//     set_dm8009_MIT(CAN_2, JOINT_RB_SEND, 0, 0, 20, 5, 0);
//   }
// } //会被多次调用

static void chassis_enable_task() {
  chassis.leg_L.ground_pid.p = CHASSIS_LEG_L0_PID_P;
  chassis.leg_R.ground_pid.p = CHASSIS_LEG_L0_PID_P;
  chassis.leg_L.ground_pid.d = CHASSIS_LEG_L0_PID_D;
  chassis.leg_R.ground_pid.d = CHASSIS_LEG_L0_PID_D;
//  is_chassis_off_ground();

  //chassis_vx_kalman_run();
}    //利用模型和传感器数据获得扭矩值

static void chassis_disable_task() {
  chassis.leg_L.wheel_torque = 0;
  chassis.leg_R.wheel_torque = 0;
  chassis.leg_L.joint_F_torque = 0;
  chassis.leg_L.joint_B_torque = 0;
  chassis.leg_R.joint_F_torque = 0;
  chassis.leg_R.joint_B_torque = 0;

  chassis.leg_L.state_variable_feedback.x = 0;
  chassis.leg_R.state_variable_feedback.x = 0;

  chassis.chassis_ctrl_info.yaw_angle_rad = chassis.imu_reference.yaw_angle;

  lk9025_set_enable(CAN_2, WHEEL_L_SEND);
  lk9025_set_enable(CAN_2, WHEEL_R_SEND);
}

float joint_position_control(pid_type_def
   *pid,float target_position, float current_position)
{
  static float output=0.0;  
    // 调用PID计算函数,计算控制输出
    Custom_PID_Calc(pid,target_position,current_position);
    output=pid->out;
    return output;
}

void dismantle(char *str)
{
    int i;
    float num;
    char buf[5]; // 用5个字符存储4位数字和结尾'\0'
    
    // 每4个字符切分一次，共3组
    for(i = 0; i < 3; i++) {
        // 复制4个字符到 buf
        strncpy(buf, str + i * 4, 4);
        buf[4] = '\0';
        num = atof(buf);
        arr[i] = num;
    }
}

float normalize_diff(float diff) {
  if(diff > 180.0f) {
      diff -= 360.0f;
  } else if(diff < -180.0f) {
      diff += 360.0f;
  }
  return diff;
}

int determine_spin_direction(float target, float current) {
  float diff = normalize_diff(target - current);
  return (diff < 0) ? 1 : 0;
}

extern void chassis_task(void const *pvParameters) {
  chassis_init();   //底盘初始化
  
  //TickType_t last_wake_time = xTaskGetTickCount();
  PID_init(&wheel_pid, PID_POSITION, wheel_PID, CHASSIS_SPIN_PID_OUT_LIMIT, CHASSIS_SPIN_PID_IOUT_LIMIT);   //不能放在while里，待查证为什么
  //以下先注释
  PID_init(&head_pid, PID_POSITION, head_PID, CHASSIS_SPIN_PID_OUT_LIMIT, 0.4);
  PID_init(&shoulder_pid, PID_POSITION, shoulder_PID, CHASSIS_SPIN_PID_OUT_LIMIT, 0.4);
  PID_init(&speed_pid, PID_POSITION, speed_PID, 20.0f, 0.03f);
  PID_init(&yaw_pid, PID_POSITION, yaw_PID, 0.35, 0.03);
  while (1) {
    //vTaskSuspendAll();
    dismantle(UARTBUF);

    if (initial_yaw == 0) {
    initial_yaw = chassis.imu_reference.yaw_angle*57.3;
    initial_head_encoder=wheel[2].encoder;
  }
    yaw_encoder_error=normalize_diff(wheel[2].encoder-initial_head_encoder);
    rc_received=local_rc_ctrl->rc.s[1];

    if (uart_received) {
      pitch_zero_offset = arr[0];
      // wheel_PID[0] = -arr[0];
      // wheel_PID[1] = -arr[1];
      // speed_PID[0] = -arr[2];
      // wheel_pid.Kp = wheel_PID[0];
      // wheel_pid.Kd = wheel_PID[1];
      // speed_pid.Kp = speed_PID[0];
      // speed_pid.Ki=speed_PID[0]/200.0f;
  }
    if (UARTBUF[0] != '\0') {
      uart_received = true;
  }
    //head_position=arr[0];
    positions[1]=257.64f;
    positions[2]=51.18f;
    // positions[1]=102.36f;
    // positions[2]=308.0f;
    //maxSpeed=arr[1];
    // spinDirection=arr[2];
    if (rc_received==1) {
      lk8010_encoder_read(CAN_2, JOINT_RF_SEND);
      osDelay(1);
      lk8010_encoder_read(CAN_2, JOINT_RB_SEND);
      osDelay(1);
      lk8010_encoder_read(CAN_1, JOINT_LF_SEND);
      osDelay(1);
      lk8010_encoder_read(CAN_1, JOINT_LB_SEND);
      osDelay(1);
      lk9025_encoder_read(CAN_1, HEAD_SEND);
      osDelay(1);
      lk8016_encoder_read(CAN_2, JOINT_PF_SEND);
      osDelay(1);
      lk8016_encoder_read(CAN_2, JOINT_PB_SEND);
  }
    get_IMU_info();
    
		chassis.chassis_ctrl_mode = CHASSIS_SPIN;
    switch (chassis.chassis_ctrl_mode) {
      case CHASSIS_SPIN:chassis.chassis_ctrl_info.v_m_per_s = 0;
        chassis.chassis_ctrl_info.yaw_angle_rad = 0;
        // ======= 外环速度 PID，输出期望姿态角度 ========
        average_velocity = (-wheel[0].angular_vel * 0.675f + wheel[1].angular_vel * 0.675f) / 2.0f;
        speed_target = local_rc_ctrl->rc.ch[3] / 110.0f;
        float pitch_target =pitch_zero_offset -joint_position_control(&speed_pid, speed_target, average_velocity);  // 期望 pitch

        // ======= 内环姿态 PID，输出转矩 ========
        float pitch_angle = chassis.imu_reference.pitch_angle * 57.3f;
        float pitch_output = joint_position_control(&wheel_pid, pitch_target, pitch_angle);

        // ======= 偏航角 PID 控制 ========
        yaw_target = local_rc_ctrl->rc.ch[0] / 3.667f;
        yaw_feedback = chassis.imu_reference.yaw_angle * 57.3f;
        float yaw_output = joint_position_control(&yaw_pid, yaw_target, yaw_feedback);  

        // ======= 合成轮子输出 ========
        chassis.leg_L.wheel_torque = pitch_output - yaw_output;
        chassis.leg_R.wheel_torque = pitch_output + yaw_output;

        break;
      default:break;
    }
    //xTaskResumeAll();
    chassis_motor_cmd_send();
    osDelay(2);
    // usart_printf("%f,%f,%f,%f,%f,%f\n",chassis.imu_reference.yaw_angle*57.3,initial_yaw,initial_head_encoder,shoulder[0].encoder,yaw_encoder_error,chassis.leg_L.joint_F_torque);
    usart_printf("%d,%f,%f,%f,%f,%f,%f,%f\n", rc_received,speed_target,error_yaw,speed_out,chassis.imu_reference.pitch_angle*57.3-5.4,chassis.imu_reference.yaw_angle*57.3,average_velocity,pid_output);
    //vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
  }
}
