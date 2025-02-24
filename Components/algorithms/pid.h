#ifndef __PID_H__
#define __PID_H__

/* 包含头文件 ----------------------------------------------------------------*/

/* 类型定义 ------------------------------------------------------------------*/
enum
{
    LLAST = 0,
    LAST,
    NOW,
    POSITION_PID,
    DELTA_PID,
};

typedef struct pid_t
{
    float p;
    float i;
    float d;

    float set;
    float get;
    float err[3];

    float pout;
    float iout;
    float dout;
    float out;

    float input_max_err;    //input max err;
    float output_deadband;  //output deadband;

    int pid_mode;
    float max_out;
    float integral_limit;

} pid_t;

typedef struct
{
    pid_t           outer_pid;
    pid_t           inter_pid;
    float           outer_ref;
    float           outer_fdb;
    float           inter_ref;
    float           inter_fdb;
} Double_PID_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void pid_init(pid_t* pid, int mode, float maxout, float intergral_limit, float kp, float ki, float kd);
float pid_calc(pid_t *pid, float get, float set);
void pid_clear(pid_t *pid);
float DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb);

#endif  // __PID_H__

