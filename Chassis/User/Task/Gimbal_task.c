#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#define MAX_SPEED 200
#define MAX_ANGLE 3400
#define MIN_ANGLE 1600

extern INS_t INS;
gimbal_t gimbal_Yaw, gimbal_Pitch; // 云台电机信息结构体

extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体

// 云台电机的初始化
static void Gimbal_loop_Init();

// 模式选择
static void mode_select();

// 云台电机的任务
static void gimbal_current_give();

// 遥控器控制云台电机
static void RC_gimbal_control();

// 锁云台模式
static void gimbal_yaw_control();

static void detel_calc(fp32 *angle);
static void detel_calc2(fp32 *angle);

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
    gimbal_Yaw.pid_parameter[0] = 85, gimbal_Yaw.pid_parameter[1] = 0, gimbal_Yaw.pid_parameter[2] = 0;
    gimbal_Yaw.pid_angle_parameter[0] = 5, gimbal_Yaw.pid_angle_parameter[1] = 0, gimbal_Yaw.pid_angle_parameter[2] = 0;
    gimbal_Yaw.angle_target = 0;

    gimbal_Pitch.pid_parameter[0] = 80, gimbal_Pitch.pid_parameter[1] = 0, gimbal_Pitch.pid_parameter[2] = 10;
    gimbal_Pitch.pid_angle_parameter[0] = 1, gimbal_Pitch.pid_angle_parameter[1] = 0, gimbal_Pitch.pid_angle_parameter[2] = 0;
    gimbal_Pitch.angle_target = 2900;

    // 初始化pid结构体
    pid_init(&gimbal_Yaw.pid, gimbal_Yaw.pid_parameter, 15000, 15000);
    pid_init(&gimbal_Yaw.pid_angle, gimbal_Yaw.pid_angle_parameter, 15000, 15000);

    pid_init(&gimbal_Pitch.pid, gimbal_Pitch.pid_parameter, 15000, 15000);
    pid_init(&gimbal_Pitch.pid_angle, gimbal_Pitch.pid_angle_parameter, 1000, 1000);
}

// 模式选择
static void mode_select()
{
    if (rc_ctrl.rc.s[0] == 1)
    {
        RC_gimbal_control();
    }
    else
    {
        gimbal_yaw_control();
    }
}

// 给电流，CAN1调试用，没板子。。。。。。
static void gimbal_current_give()
{
    gimbal_Yaw.motor_info.set_current = pid_calc(&gimbal_Yaw.pid, gimbal_Yaw.motor_info.rotor_speed, gimbal_Yaw.speed_target);
    gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid, gimbal_Pitch.motor_info.rotor_speed, gimbal_Pitch.speed_target);
    set_motor_current_gimbal(1, gimbal_Yaw.motor_info.set_current, 0, 0, 0);
    set_motor_current_gimbal2(1, 0, 0, gimbal_Pitch.motor_info.set_current, 0);
}

// 遥控器控制云台电机
static void RC_gimbal_control()
{
    if (rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0] <= 660)
    {
        gimbal_Yaw.speed_target = -rc_ctrl.rc.ch[0] / 660.0 * MAX_SPEED;
        gimbal_Yaw.angle_target = INS.Yaw; // 保證切換模式時不會突變
    }
    else
    {
        gimbal_Yaw.speed_target = 0;
    }

    // Pitch轴
    // 1600 < gimbal_Pitch.angle_target < 3400
    if (rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1] <= 660)
    {
        gimbal_Pitch.angle_target += rc_ctrl.rc.ch[1] / 660.0 * 1;
        detel_calc2(&gimbal_Pitch.angle_target);

        gimbal_Pitch.speed_target = pid_calc(&gimbal_Pitch.pid_angle, gimbal_Pitch.motor_info.rotor_angle, gimbal_Pitch.angle_target);
    }
}

// 锁云台模式
static void gimbal_yaw_control()
{
    if (rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0] <= 660)
    {
        gimbal_Yaw.angle_target += rc_ctrl.rc.ch[0] / 660.0 * (-0.1);

        detel_calc(&gimbal_Yaw.angle_target);

        gimbal_Yaw.speed_target = gimbal_PID_calc(&gimbal_Yaw.pid_angle, INS.Yaw, gimbal_Yaw.angle_target);
    }
    else
    {
        gimbal_Yaw.angle_target = 0;
    }

    if (rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1] <= 660)
    {
        gimbal_Pitch.angle_target += rc_ctrl.rc.ch[1] / 660.0 * 1;
        detel_calc2(&gimbal_Pitch.angle_target);

        gimbal_Pitch.speed_target = pid_calc(&gimbal_Pitch.pid_angle, gimbal_Pitch.motor_info.rotor_angle, gimbal_Pitch.angle_target);
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

static void detel_calc2(fp32 *angle)
{
    if (*angle > 8192)
        *angle -= 8192;

    else if (*angle < 0)
        *angle += 8192;

    if (*angle > MAX_ANGLE)
        *angle = MAX_ANGLE;

    else if (*angle < MIN_ANGLE)
        *angle = MIN_ANGLE;
}
