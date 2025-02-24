/* 包含头文件 ----------------------------------------------------------------*/
#include "chassis_function.h"
#include "infantry_def.h"
#include "referee_system.h"
#include "detect_task.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define MAX_WHEEL_RPM   M3508_MAX_RPM

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f
#define WARNING_POWER_BUFF  50.0f
#define MAX_TOTAL_CURRENT_LIMIT         64000.0f    //16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

/* 私有变量 ------------------------------------------------------------------*/
fp32 yu;
extern ChassisHandle_t chassis_handle;
fp32 spin_rate[9]={1.0,1.5,2.0,2.8,3.0,4.2,4.7,5.4,5.8};
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void Chassis_MoveTransform(ChassisHandle_t* chassis_handle, fp32* chassis_vx, fp32* chassis_vy)
{
    static fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

    sin_yaw = arm_sin_f32(chassis_handle->gimbal_yaw_ecd_angle / RADIAN_COEF);
    cos_yaw = arm_cos_f32(chassis_handle->gimbal_yaw_ecd_angle / RADIAN_COEF);

    *chassis_vx = cos_yaw * chassis_handle->vx + sin_yaw * chassis_handle->vy;
    *chassis_vy =-sin_yaw * chassis_handle->vx + cos_yaw * chassis_handle->vy;

}

void Mecanum_Calculate(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw)
{
    static float rotate_ratio_fr;
    static float rotate_ratio_fl;
    static float rotate_ratio_bl;
    static float rotate_ratio_br;
    static float wheel_rpm_ratio;

    rotate_ratio_fl = ((chassis_handle->structure.wheelbase + chassis_handle->structure.wheeltrack)/2.0f \
            - chassis_handle->structure.rotate_x_offset - chassis_handle->structure.rotate_y_offset)/RADIAN_COEF;
    rotate_ratio_fr = ((chassis_handle->structure.wheelbase + chassis_handle->structure.wheeltrack)/2.0f \
            - chassis_handle->structure.rotate_x_offset + chassis_handle->structure.rotate_y_offset)/RADIAN_COEF;
    rotate_ratio_bl = ((chassis_handle->structure.wheelbase + chassis_handle->structure.wheeltrack)/2.0f \
            + chassis_handle->structure.rotate_x_offset - chassis_handle->structure.rotate_y_offset)/RADIAN_COEF;
    rotate_ratio_br = ((chassis_handle->structure.wheelbase + chassis_handle->structure.wheeltrack)/2.0f \
            + chassis_handle->structure.rotate_x_offset + chassis_handle->structure.rotate_y_offset)/RADIAN_COEF;

    wheel_rpm_ratio = 60.0f/(chassis_handle->structure.wheel_perimeter * M3508S_REDUCTION_RATIO);

    VAL_LIMIT(chassis_vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(chassis_vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(chassis_vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);  //deg/s

    fp32 wheel_rpm[4];
    fp32 max = 0;

    wheel_rpm[0] = ( chassis_vx + chassis_vy - chassis_vw * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[1] = (-chassis_vx + chassis_vy - chassis_vw * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[2] = ( chassis_vx - chassis_vy - chassis_vw * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[3] = (-chassis_vx - chassis_vy - chassis_vw * rotate_ratio_br) * wheel_rpm_ratio;

    //find max item
    for (uint8_t i = 0; i < 4; i++)
    {
        if (fabs(wheel_rpm[i]) > max)
        {
            max = fabs(wheel_rpm[i]);
        }
    }

    //equal proportion
//    if (max > MAX_WHEEL_RPM)
//    {
//        float rate = MAX_WHEEL_RPM / max;
//        for (uint8_t i = 0; i < 4; i++)
//        {
//            wheel_rpm[i] *= rate;
//        }
//    }
		for (uint8_t i = 0; i < 4; i++)
		{
			if(RefereeSystem_RobotState_Pointer()->chassis_power_limit==100)           //¸ù¾Ý¶ÔÓ¦¹¦ÂÊ¸³Öµ²»Í¬×ªËÙÏµÊý
			{
				if(chassis_handle->super_flag==1)
				{
					wheel_rpm[i]=wheel_rpm[i]*spin_rate[8];
				}
				else if(chassis_handle->super_flag==0)
				{
					wheel_rpm[i]=wheel_rpm[i]*spin_rate[6];
				}
			}
			else
			{
				if(chassis_handle->super_flag==1)
				{
					wheel_rpm[i]=wheel_rpm[i]*spin_rate[7];
				}
				else if(chassis_handle->super_flag==0)
				{
					wheel_rpm[i]=wheel_rpm[i]*spin_rate[6];
				}
		 }
	 }
    memcpy(chassis_handle->wheel_rpm, wheel_rpm, 4 * sizeof(fp32));
}

void Chassis_ControlCalc(ChassisHandle_t* chassis_handle)
{
    static float chassis_vx = 0.0f, chassis_vy = 0.0f;
    Chassis_MoveTransform(chassis_handle, &chassis_vx, &chassis_vy);

    Mecanum_Calculate(chassis_handle, chassis_vx, chassis_vy, chassis_handle->vw);
}

void Chassis_LimitPower(ChassisHandle_t* chassis_handle)
{
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
	  fp32 chassis_power_true =0.0f;
    fp32 max_chassis_power = 0.0f;
   
	  
    uint8_t robot_id = RefereeSystem_GetRobotID();

    if (robot_id == 0 || CheckDeviceIsOffline(OFFLINE_REFEREE_SYSTEM))
    {
        total_current_limit = 8000;
    }
    else if (robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER)
    {
        total_current_limit = MAX_TOTAL_CURRENT_LIMIT;
    }
    else
    {
//      chassis_power_true=chassis_handle->Chassis_super_power+chassis_power_comps;
        chassis_power_buffer = RefereeSystem_PowerHeatData_Pointer()->chassis_power_buffer;
        max_chassis_power = RefereeSystem_RobotState_Pointer()->chassis_power_limit;
               
            if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            fp32 power_scale;
            if(chassis_power_buffer > 5.0f)
            {
                //scale down WARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                //only left 10% of WARNING_POWER_BUFF
                power_scale = 5.0f / WARNING_POWER_BUFF;
            }
            //scale down
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            //power > WARNING_POWER
            if(chassis_power_true > WARNING_POWER)
            {
                fp32 power_scale;
                //power < 80w
                if(chassis_power_true < max_chassis_power)
                {
                    //scale down
                    power_scale = (max_chassis_power - chassis_power_true) / (max_chassis_power - WARNING_POWER);

                }
                //power > 80w
                else
                {
                    power_scale = 0.0f;
                }

                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //power < WARNING_POWER
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    }
        /* 电流输出控制 */
    //calculate the original motor current set
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_handle->chassis_motor[i].current_set);
    }

//  if(chassis_handle->super_flag==0)
//	{
    if(total_current > total_current_limit)
    {
			  fp32 current_scale = total_current_limit / total_current;            //系数<1减速，系数>1加速
        chassis_handle->chassis_motor[0].current_set *= current_scale;
        chassis_handle->chassis_motor[1].current_set *= current_scale;
        chassis_handle->chassis_motor[2].current_set *= current_scale;
        chassis_handle->chassis_motor[3].current_set *= current_scale;
    }
//	}
}

