/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_function.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
fp32 Gimbal_PID_Calc(Gimbal_PID_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb)
{
    pid->angle_ref = angle_ref;
    pid->angle_fdb = angle_fdb;
    pid_calc(&pid->outer_pid, pid->angle_fdb, pid->angle_ref);
    pid->speed_ref = pid->outer_pid.out;
    pid->speed_fdb = speed_fdb;
    pid_calc(&pid->inter_pid, pid->speed_fdb, pid->speed_ref);
    return pid->inter_pid.out;
}

void Gimbal_PID_Clear(Gimbal_PID_t* pid)
{
    pid_clear(&pid->outer_pid);
    pid_clear(&pid->inter_pid);
}

void GimbalMotorChangeProtect(GimbalMotor_t* motor)
{
    if (motor->last_mode != motor->mode)
    {
        if(motor->mode == RAW_VALUE_MODE)
        {
            motor->given_value = motor->current_set;
        }
        else if (motor->mode == GYRO_MODE)
        {
            motor->given_value = motor->sensor.gyro_angle;
        }
        else if (motor->mode == ENCONDE_MODE)
        {
            motor->given_value = motor->sensor.relative_angle;
        }
    }
    motor->last_mode = motor->mode;
}

void GimbalMotorControl(GimbalMotor_t* motor)
{
    GimbalMotorChangeProtect(motor);
    if (motor->mode == RAW_VALUE_MODE)
    {
        motor->current_set = motor->given_value;
        Gimbal_PID_Clear(&motor->pid);
    }
    else if(motor->mode == GYRO_MODE)
    {
        motor->current_set = Gimbal_PID_Calc(&motor->pid,
                                             motor->given_value,
                                             motor->sensor.gyro_angle,
                                             motor->sensor.palstance);
    }
    else if(motor->mode == ENCONDE_MODE)
    {
        motor->current_set = Gimbal_PID_Calc(&motor->pid,
                                             motor->given_value,
                                             motor->sensor.relative_angle,
                                             motor->sensor.palstance);
    }
}

void AimReset(AimCalcData_t* data,fp32 angle_raf);
void AimCalc(AimCalcData_t* data, fp32 angle, fp32 time,fp32 angle_raf);
void AimReset(AimCalcData_t* data,fp32 angle_raf)
{
    memset(data, 0, sizeof(AimCalcData_t));
}
/*
void AimCalc(AimCalcData_t* data, fp32 angle, fp32 time,fp32 angle_raf)
{
    data->times += time;
    if (data->times < 400)
    {
        time= 0;
    }
    if(data->times == 0)
    {
        data->angle[1] = angle_raf;
    }

    data->angle[0] = angle;
    fp32 P = data->angle[1] + 0.15f * (data->angle[0] - data->angle[1]); // 预判角度
    fp32 gv = data->gv[1] + 0.1f * (P - data->angle[1]);
    fp32 acc = 0.707f * (gv - data->gv[1]);
    data->acc[0] = data->acc[0] + 0.01f * (acc - data->acc[0]);

    VAL_LIMIT(gv, -0.3f, 0.3f);

    data->angle[1] = P + gv + 0.5f*data->acc[0];
    data->gv[1] = gv - acc + data->acc[0];

    data->out = P + time * gv + 0.5f * time * time * data->acc[0];

}
*/
void AimCalc(AimCalcData_t* data, fp32 angle, fp32 time,fp32 angle_raf)
{
    if(data->times == 0)
    {
        data->angle[1] = angle_raf;
    }
    data->times += time;
    if (data->times < 10)
    {
        time= 0;
    }
    
    data->angle[0] = angle;
    fp32 P = data->angle[1] + 0.15f * (data->angle[0] - data->angle[1]); // 预判角度
    fp32 gv = data->gv[1] + 0.1f * (P - data->angle[1]);
    fp32 acc = 0.707f * (gv - data->gv[1]);
    data->acc[0] = data->acc[0] + 0.01f * (acc - data->acc[0]);

    VAL_LIMIT(gv, -0.3f, 0.3f);

    data->angle[1] = P + gv + 0.5f*data->acc[0];
    data->gv[1] = gv - acc + data->acc[0];

    data->out = P + time * gv + 0.5f * time * time * data->acc[0];

}

 fp32 AngleTransform(fp32 target_angle, fp32 gyro_angle)
{
    float offset = 0, now = 0, target = 0;

    ANGLE_LIMIT_360(target, target_angle);
    ANGLE_LIMIT_360(now, gyro_angle);

    offset = target - now;
    if (offset > 180)
    {
        offset = offset - 360;
    }
    else if (offset < -180)
    {
        offset = offset + 360;
    }
    return gyro_angle + offset;
}

fp32 pitch_back(fp32 dcf)
{
    static fp32 back_num = 0;
    back_num = (0.17016 * pow(dcf,3)) - (1.47865 * pow(dcf,2)) + (5.02075 * dcf) - 4.90086;
    VAL_LIMIT(back_num,0,3);
    return back_num;
}

fp32 Target_Speed_Calc(Speed_Calc_Data_t *Speed,uint32_t time,fp32 position)
{
    Speed->delay_cnt++;
    
    if(time != Speed->last_time)
    {
        Speed->speed = ((position - Speed->last_position) / (time-Speed->last_time))*2;
        
        Speed->processed_speed = Speed->speed;
        Speed->last_time = time;
        Speed->last_position = position;
        Speed->last_speed = Speed->speed;
        Speed->delay_cnt = 0;
    }
    
    if(Speed->delay_cnt > 300)
    {
        Speed->processed_speed = 0;
    }
    return Speed->processed_speed;
}

fp32 Auto_Aim_RAMP(fp32 final, fp32 now, fp32 ramp)
{
    fp32 buffer=0;
    
    buffer = final - now;
    
    if(buffer > 0)   
    {
        if(buffer > ramp)
        {
                now += ramp;
        }
        else
        {
            now += buffer;
        }
    }
    else
    {
        if(buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }
    
    return now;
}
