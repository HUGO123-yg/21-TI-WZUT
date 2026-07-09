#include "pid.h"

pid_casade_t balance_control;
pid_casade_t steer_balance_control;

pid_struct_t speed_control;
pid_struct_t angle_control;
pid_struct_t ang_v_control;

void pid_init(pid_struct_t *pid, float kp, float ki, float kd, float in_max, float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->in_max  = in_max;
  pid->out_max = out_max;
  pid->in_min  = -in_max;
  pid->out_min = -out_max;

  pid->ref     = 0.0f;
  pid->fdb     = 0.0f;
  pid->err[0]  = 0.0f;
  pid->err[1]  = 0.0f;
  pid->p_out   = 0.0f;
  pid->i_out   = 0.0f;
  pid->d_out   = 0.0f;
  pid->output  = 0;
}

float pid_calc(pid_struct_t *pid, float ref, float fdb)
{
    pid->ref = ref;
    pid->fdb = fdb;
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->ref - pid->fdb;

    pid->p_out  = pid->kp * pid->err[0];
    pid->i_out += pid->ki * pid->err[0];
    pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
    pid->i_out=LIMIT_MIN_MAX(pid->i_out, pid->in_min, pid->in_max);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    pid->output=LIMIT_MIN_MAX(pid->output, pid->out_min, pid->out_max);
    return pid->output;
}

float pid_calc_in(pid_struct_t *pid, float ref, float fdb)
{
    float error_prev = pid->err[0];     // ������һ�����
    float error_prev2 = pid->err[1];    // �������ϴ����

    pid->ref = ref;
    pid->fdb = fdb;
    pid->err[1] = error_prev;           // �������ϴ����
    pid->err[0] = pid->ref - pid->fdb;  // ���㵱ǰ���

    // ����ʽPID����
    float delta_output = pid->kp * (pid->err[0] - error_prev) +
                         pid->ki * pid->err[0] +
                         pid->kd * (pid->err[0] - 2 * error_prev + error_prev2);

    // �������������Χ����ѡ��
    delta_output = LIMIT_MIN_MAX(delta_output, pid->in_min, pid->in_max);

    // ���������
    pid->output += delta_output;

    // �������������Χ
    pid->output = LIMIT_MIN_MAX(pid->output, pid->out_min, pid->out_max);

    return pid->output;
}
