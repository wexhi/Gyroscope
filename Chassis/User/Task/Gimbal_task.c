#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#define MAX_SPEED 200

extern INS_t INS;
gimbal_t gimbal_Yaw, gimbal_Pitch;
fp32 err_yaw_angle;

extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体

// 云台电机的初始化
static void Gimbal_loop_Init();

// 模式选择
static void mode_select();

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
        mode_select();
        gimbal_current_give();
        osDelay(1);
    }
}

// 云台电机的初始化
static void Gimbal_loop_Init()
{
    // 初始化pid参数
    gimbal_Yaw.pid_parameter[0] = 20, gimbal_Yaw.pid_parameter[1] = 0, gimbal_Yaw.pid_parameter[2] = 0;
    gimbal_Yaw.pid_angle_parameter[0] = 5, gimbal_Yaw.pid_angle_parameter[1] = 0, gimbal_Yaw.pid_angle_parameter[2] = 50;
    gimbal_Yaw.angle_target = 0;

    gimbal_Pitch.pid_parameter[0] = 20, gimbal_Pitch.pid_parameter[1] = 0, gimbal_Pitch.pid_parameter[2] = 0;
    gimbal_Pitch.pid_angle_parameter[0] = 5, gimbal_Pitch.pid_angle_parameter[1] = 0, gimbal_Pitch.pid_angle_parameter[2] = 50;
    gimbal_Pitch.angle_target = 0;

    // 初始化pid结构体
    pid_init(&gimbal_Yaw.pid, gimbal_Yaw.pid_parameter, 15000, 15000);
    pid_init(&gimbal_Yaw.pid_angle, gimbal_Yaw.pid_angle_parameter, 15000, 15000);

    pid_init(&gimbal_Pitch.pid, gimbal_Pitch.pid_parameter, 15000, 15000);
    pid_init(&gimbal_Pitch.pid_angle, gimbal_Pitch.pid_angle_parameter, 15000, 15000);
}

// 模式选择
static void mode_select()
{
    if (rc_ctrl.rc.s[0] == 1)
    {
        gimbal_yaw_control();
    }
    else
    {
        RC_gimbal_control();
    }
}

// 给电流，CAN1调试用，没板子。。。。。。
static void gimbal_current_give()
{
    gimbal_Yaw.motor_info.set_current = pid_calc(&gimbal_Yaw.pid, gimbal_Yaw.motor_info.rotor_speed, gimbal_Yaw.speed_target);
    set_motor_current_gimbal(1, gimbal_Yaw.motor_info.set_current, 0, 0, 0);
}

// 遥控器控制云台电机
static void RC_gimbal_control()
{
    if (rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0] <= 660)
    {
        gimbal_Yaw.speed_target = -rc_ctrl.rc.ch[0] / 660.0 * MAX_SPEED;
    }
    else
    {
        gimbal_Yaw.speed_target = 0;
    }
}

// yaw轴控制电机
static void gimbal_yaw_control()
{
    if (rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0] <= 660)
    {
        gimbal_Yaw.angle_target = -(gimbal_Yaw.angle_target + rc_ctrl.rc.ch[0] / 660.0 * 0.3);

        detel_calc(&gimbal_Yaw.angle_target);

        err_yaw_angle = gimbal_Yaw.angle_target - INS.Yaw;

        detel_calc(&err_yaw_angle);

        if (err_yaw_angle < -0.1 || err_yaw_angle > 0.1)
        {
            gimbal_Yaw.speed_target = gimbal_PID_calc(&gimbal_Yaw.pid_angle, INS.Yaw, gimbal_Yaw.angle_target);
        }
        else
        {
            gimbal_Yaw.speed_target = 0;
        }
    }
    else
    {
        gimbal_Yaw.angle_target = 0;
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
