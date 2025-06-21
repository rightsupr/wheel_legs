/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

void pid_init(struct Pid *pid, float max_out, float intergral_limit,
              float kp, float ki, float kd)
{
    pid->integral_limit = intergral_limit;
    pid->max_output     = max_out;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

    pid->err[NOW]=0;
    pid->err[LAST]=0;
    pid->set=0;
    pid->get=0;
    pid->pout=0;
    pid->iout=0;
    pid->dout=0;
    pid->excess=0;
}
/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief PID计算函数 (自定义实现)
  * @param pid       PID结构体指针
  * @param setpoint  设定值
  * @param feedback  反馈值
  * @return fp32     PID输出值（扭矩）
  */
 fp32 Custom_PID_Calc(pid_type_def *pid, fp32 setpoint, fp32 feedback) {
  if (pid == NULL) {
      return 0.0f;
  }

  // 更新历史误差（向前滚动）
  pid->error[2] = pid->error[1];  // 上次的error[1]变为error[2]
  pid->error[1] = pid->error[0];  // 当前的error[0]变为error[1]
  pid->error[0] = setpoint - feedback; // 计算新误差

  // 更新设定值和反馈值（可选，根据需求）
  pid->set = setpoint;
  pid->fdb = feedback;

  if (pid->mode == PID_POSITION) {
      // ========= 位置式PID =========
      // 比例项
      pid->Pout = pid->Kp * pid->error[0];
      
      // 积分项（带抗饱和）
      pid->Iout += pid->Ki * pid->error[0];
      // 积分限幅
      if (pid->Iout > pid->max_iout) {
          pid->Iout = pid->max_iout;
      } else if (pid->Iout < -pid->max_iout) {
          pid->Iout = -pid->max_iout;
      }
      
      // 微分项（使用当前误差与上一次误差的差值）
      pid->Dbuf[2] = pid->Dbuf[1];  // 历史微分值滚动
      pid->Dbuf[1] = pid->Dbuf[0];
      pid->Dbuf[0] = pid->error[0] - pid->error[1];  // 当前微分差分
      pid->Dout = pid->Kd * pid->Dbuf[0];
      
      // 合成输出
      pid->out = pid->Pout + pid->Iout + pid->Dout;
      
  } else if (pid->mode == PID_DELTA) {
      // ========= 增量式PID =========
      // 比例项（误差变化量）
      pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
      
      // 积分项（当前误差）
      fp32 I_temp = pid->Ki * pid->error[0];
      // 积分限幅后再累加（防止单次积分过大）
      if (I_temp > pid->max_iout) {
          I_temp = pid->max_iout;
      } else if (I_temp < -pid->max_iout) {
          I_temp = -pid->max_iout;
      }
      pid->Iout += I_temp;
      
      // 微分项（误差变化量的变化）
      pid->Dbuf[2] = pid->Dbuf[1];
      pid->Dbuf[1] = pid->Dbuf[0];
      pid->Dbuf[0] = pid->error[0] - 2.0f * pid->error[1] + pid->error[2];  // 二阶差分
      pid->Dout = pid->Kd * pid->Dbuf[0];
      
      // 输出为增量值（需累加）
      pid->out += pid->Pout + pid->Iout + pid->Dout;
  }

  // 总输出限幅
  if (pid->out > pid->max_out) {
      pid->out = pid->max_out;
  } else if (pid->out < -pid->max_out) {
      pid->out = -pid->max_out;
  }

  return pid->out;
}
 
/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
