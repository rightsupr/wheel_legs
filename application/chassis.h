#ifndef CHASSIS_H
#define CHASSIS_H

#include "stdbool.h"
#include "wheel.h"
#include "pid.h"
#define REMOTE 0

#define CHASSIS_PERIOD 5

/*******************************************************************************
 *                                    Limit                                    *
 *******************************************************************************/
#define MAX_CHASSIS_VX_SPEED 1.8f
#define MAX_CHASSIS_VW_TORQUE 0.5f
#define MAX_CHASSIS_YAW_INCREMENT 0.02f
#define MIN_L0 0.13f
#define MAX_L0 0.40f
#define DEFAULT_L0 0.22f
#define MAX_PITCH 0.174533f
#define MIN_PITCH (-0.174533f)
#define MAX_ROLL 0.12f
#define MIN_ROLL (-0.12f)
#define MAX_WHEEL_TORQUE 10.f
#define MIN_WHEEL_TORQUE (-10.f)
#define MAX_JOINT_TORQUE 40.f
#define MIN_JOINT_TORQUE (-40.f)

#define WHEEL_THETA_LIMIT      1.5f
#define WHEEL_THETA_DOT_LIMIT  0.5f
#define WHEEL_X_LIMIT          0.f
#define WHEEL_X_DOT_LIMIT      1.0f
#define WHEEL_PHI_LIMIT        1.0f
#define WHEEL_PHI_DOT_LIMIT    0.5f

#define JOINT_THETA_LIMIT      15.5f
#define JOINT_THETA_DOT_LIMIT  20.5f
#define JOINT_X_LIMIT          0.f
#define JOINT_X_DOT_LIMIT      10.0f
#define JOINT_PHI_LIMIT        10.5f
#define JOINT_PHI_DOT_LIMIT    10.0f

extern char UARTBUF[13];
extern uint8_t len;

//底盘物理参数
struct ChassisPhysicalConfig {
  float wheel_radius;    //轮子半径
  float body_weight;     //身体重量
  float wheel_weight;    //轮子重量
  float mechanical_leg_limit_angle;   //腿的角度限制值

  float l1, l2, l3, l4, l5;
};
//用于表示机器人底盘控制模式
enum ChassisCtrlMode {
  CHASSIS_DISABLE = 1,
  CHASSIS_ENABLE,
  CHASSIS_INIT,
  CHASSIS_JUMP,
  CHASSIS_SPIN,
  CHASSIS_SIDEWAYS,
};
//表示两个腿的索引
enum LegIndex {
  R = 1,
  L = 0,
};
//表示一个跳跃动作的不同状态
enum JumpState {
  NOT_READY,  //跳跃尚未准备好
  READY,      //已准备
  STRETCHING, //伸展
  SHRINKING,  //收缩
  STRETCHING_AGAIN,  //再次伸展阶段（可能是起跳动作的一部分）
  FALLING,    //下落阶段
  LANDING,    //着陆阶段
};
//监控腿部的实时状态，以便进行相应的决策和动作调整。
struct LegFlag {
  bool OFF_GROUND_FLAG;  //离地
  bool IMPACT_FLAG;      //受到冲击
};
//惯性测量单元（IMU）相关的数据
struct IMUReference {
  //三轴角度
  float pitch_angle;
  float yaw_angle;
  float roll_angle;

  //三轴角速度
  float pitch_gyro;
  float yaw_gyro;
  float roll_gyro;

  //三轴加速度
  float ax;
  float ay;
  float az;

  //三轴滤波后的加速度
  float ax_filtered;
  float ay_filtered;
  float az_filtered;
  //机器人绕 Z 轴的角速度
  float robot_az;
};
//底盘运动控制相关的参数
struct ChassisCtrlInfo {
  float v_m_per_s;    //线速度
  float pitch_angle_rad; //俯仰角
  float yaw_angle_rad;   //偏航角
  float roll_angle_rad;  //横滚角
  float height_m;        //高度
  float spin_speed;      //旋转速度
};
//描述系统状态变量
struct StateVariable {
  float theta;//角度
  float theta_last;//上一时刻角度
  float theta_dot;//角速度
  float theta_dot_last;//上一时刻角速度
  float theta_ddot;//角加速度
  float x;
  float x_dot;
  float x_dot_last;
  float x_ddot;
  float phi;
  float phi_dot;
};
//腿部系统的抽象模型，整合了控制、反馈和状态等信息
struct Leg {
  enum LegIndex leg_index;  //左腿还是右腿
  struct LegFlag leg_flag;  //当前状态（离地与受冲击）
  struct StateVariable state_variable_feedback;  //实时反馈的腿部状态
  struct StateVariable state_variable_set_point; //系统设定的目标状态
  struct StateVariable state_variable_error;     //反馈与目标状态的误差
  struct StateVariable state_variable_wheel_out; //轮子相关输出状态
  struct StateVariable state_variable_joint_out; //关节相关输出状态
  // struct VMC vmc;  //VMC数据结构（包含内容待补充）
  struct Pid ground_pid;    //地面PID（包括PID参数、反馈目标误差、PID输出值）
  struct Pid offground_pid; //离地PID
  // struct KalmanFilter vx_kalman; //对横向速度（vx）进行卡尔曼滤波，更精确的速度估计（待确认）
  float kalman_measure[2];  //卡尔曼滤波器的观测数据输入
  float *kalman_result;  //卡尔曼滤波器的输出结果
  float L0_set_point; //腿部初始长度的目标值（待确认）
  float wheel_torque; //轮子的扭矩输出
  float joint_F_torque; //前关节和后关节的扭矩输出（由于他是在髋关节前后两个关节）
  float joint_B_torque;
  float joint_virtual_torque; //虚拟关节扭矩（用于在模型计算）
  float Fn; //正向力，用于接触力检测（待确认）
};
//底盘综合性数据
struct Chassis {
  enum ChassisCtrlMode chassis_ctrl_mode;  //控制模式
  enum ChassisCtrlMode chassis_ctrl_mode_last; //上一时刻控制模式，判断是否需要特殊处理
  enum JumpState jump_state;  //跳跃状态
  struct ChassisCtrlInfo chassis_ctrl_info; //底盘运动控制相关的参数
  struct IMUReference imu_reference;  //惯性测量单元（IMU）相关的数据
  struct Leg leg_L;
  struct Leg leg_R;
  struct Pid chassis_vw_speed_pid; //底盘速度（线速度与角速度）控制
  struct Pid chassis_vw_current_pid; //用于底盘电流控制
  struct Pid chassis_spin_pid;  //用于底盘旋转控制
  struct Pid chassis_roll_pid;  //底盘滚转控制
  struct Pid chassis_leg_coordination_pid;  //腿部和底盘协调控制
  bool is_chassis_offground; //是否离地
  float wheel_turn_torque;  //轮子扭矩输出（感觉冗余了，在leg处已经定义）
};

void chassis_init();
//结构体类型和函数返回值，表示这些函数的返回值类型是前面定义的结构体类型。
struct ChassisFeedbackInfo get_chassis_feedback_info();  //似乎找不到这个结构体

struct Chassis *get_chassis();

bool is_chassis_off_ground();

extern void chassis_task(void const *pvParameters);

float joint_position_control(pid_type_def *pid,float target_position, float current_position);

void dismantle(char *str);

float normalize_diff(float diff);

int determine_spin_direction(float target, float current);
#endif //CHASSIS_H
