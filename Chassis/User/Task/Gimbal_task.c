#include "Gimbal_task.h"
#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#define MAX_SPEED 200

extern INS_t INS;
extern motor_info_t motor_info_chassis[8]; // 电机信息结构体[3]为云台电机
gimbal_t gimbal;
fp32 err_yaw_angle;

extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体

// 云台电机的初始化
static void Gimbal_loop_Init();

// 云台电机的任务
static void gimbal_current_give();

// 遥控器控制云台电机
static void RC_gimbal_control();

// yaw轴控制电机
static void gimbal_yaw_control();

static void detel_calc(fp32 *angle);


void Gimbal_task(void const *pvParameters)
{
    Gimbal_loop_Init();
    for (;;)
    {
        // RC_gimbal_control();
        gimbal_yaw_control();
        gimbal_current_give();
        osDelay(1);
    }
}

// 云台电机的初始化
static void Gimbal_loop_Init()
{
    // 初始化pid参数
    gimbal.pid_parameter[0] = 20;
    gimbal.pid_parameter[1] = 0;
    gimbal.pid_parameter[2] = 0;

    gimbal.pid_angle_parameter[0] = 10;
    gimbal.pid_angle_parameter[1] = 0;
    gimbal.pid_angle_parameter[2] = 0;

    gimbal.motor_info = motor_info_chassis[3]; // 云台电机的信息结构体
    // gimbal.init_angle = gimbal.motor_info.rotor_angle * 360 / 8192;
    gimbal.init_angle = 0;

    // 初始化pid结构体
    pid_init(&gimbal.pid, gimbal.pid_parameter, 6000, 6000);           // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
    pid_init(&gimbal.pid_angle, gimbal.pid_angle_parameter, 360, 360); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
}

// 给电流，CAN1调试用，没板子。。。。。。
static void gimbal_current_give()
{
    gimbal.motor_info = motor_info_chassis[3];
    gimbal.motor_info.set_current = pid_calc(&gimbal.pid, gimbal.motor_info.rotor_speed, gimbal.speed_target);
    set_motor_current_can1(0, 0, 0, 0, gimbal.motor_info.set_current);
}

// 遥控器控制云台电机
static void RC_gimbal_control()
{
    if (rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1] <= 660)
    {
        gimbal.speed_target = rc_ctrl.rc.ch[1] / 660.0 * MAX_SPEED;
    }
    else
    {
        gimbal.speed_target = 0;
    }
}

// yaw轴控制电机
static void gimbal_yaw_control()
{
    gimbal.motor_info = motor_info_chassis[3];
    if (rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1] <= 660)
    {
        gimbal.angle_target = gimbal.init_angle + rc_ctrl.rc.ch[1] / 660.0 * 180;

        detel_calc(&gimbal.angle_target);

        err_yaw_angle = gimbal.angle_target - INS.Yaw;

        if (err_yaw_angle < -1 || err_yaw_angle > 1)
        {
            gimbal.speed_target = gimbal_PID_calc(&gimbal.pid_angle, INS.Yaw, gimbal.angle_target);
        }
        else
        {
            gimbal.speed_target = 0;
        }
    }
    else
    {
        gimbal.angle_target = 0;
    }
}

static void detel_calc(fp32 *angle)
{
    if (*angle > 360)
    {
        *angle -= 360;
    }

    else if (*angle < 0)
    {
        *angle += 360;
    }
}

