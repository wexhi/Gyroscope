#include "Gimbal_task.h"
#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#define MAX_SPEED 200

extern INS_t INS;
extern motor_info_t motor_info_chassis[8]; // 电机信息结构体[4]为云台电机
gimbal_t gimbal;
fp32 err_yaw_angle;
uint8_t mode_flag = 0;

extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体

// 云台电机的初始化
static void Gimbal_loop_Init();

// 模式选择
static void mode_select();

// 云台电机的任务
static void gimbal_current_give();

// 遥控器控制云台电机
static void RC_gimbal_control();

// 云台旋转固定角度
static void gimbal_turn_angle(fp32 angle);

// yaw轴控制电机
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
    gimbal.pid_parameter[0] = 30;
    gimbal.pid_parameter[1] = 0;
    gimbal.pid_parameter[2] = 0;

    gimbal.pid_angle_parameter[0] = 60;
    gimbal.pid_angle_parameter[1] = 0;
    gimbal.pid_angle_parameter[2] = 0;

    gimbal.motor_info = motor_info_chassis[4]; // 云台电机的信息结构体
    gimbal.speed_target = 0;
    // gimbal.angle_target = 0;
    gimbal.angle_target = gimbal.motor_info.real_angle;

    // 初始化pid结构体
    pid_init(&gimbal.pid, gimbal.pid_parameter, 16000, 16000);
    pid_init(&gimbal.pid_angle, gimbal.pid_angle_parameter, 15000, 15000);
}

// 模式选择
static void mode_select()
{
    if (rc_ctrl.rc.s[0] == 1) // BLUE LED
    {
        gimbal_turn_angle(45);
        mode_flag = 1;
    }
    if (rc_ctrl.rc.s[0] == 2) // GREEN LED
    {
        RC_gimbal_control();
        mode_flag = 2;
    }
    if (rc_ctrl.rc.s[0] == 3) // RED LED
    {
        // gimbal_yaw_control();
        mode_flag = 3;
    }
}

// 给电流，CAN1调试用，没板子。。。。。。
static void gimbal_current_give()
{
    gimbal.motor_info = motor_info_chassis[4];
    gimbal.motor_info.set_current = pid_calc(&gimbal.pid, gimbal.motor_info.rotor_speed, gimbal.speed_target);
    set_gimbal_current(1, gimbal.motor_info.set_current, 0, 0, 0);
}

// 遥控器控制云台电机
static void RC_gimbal_control()
{
    if (mode_flag != 2)
    {
        gimbal.angle_target = gimbal.motor_info.real_angle;
    }

    if (rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1] <= 660)
    {
        // gimbal.speed_target = rc_ctrl.rc.ch[1] / 660.0 * MAX_SPEED;
        gimbal.angle_target += rc_ctrl.rc.ch[1] / 660.0 * 0.3;
        detel_calc2(&gimbal.angle_target);
        gimbal.speed_target = gimbal_PID_calc(&gimbal.pid_angle, gimbal.motor_info.real_angle, gimbal.angle_target);
    }
    else
    {
        gimbal.speed_target = 0;
    }
}

// 云台旋转固定角度
static void gimbal_turn_angle(fp32 angle)
{
    if (mode_flag != 1)
    {
        gimbal.angle_target = gimbal.motor_info.real_angle + angle;
    }

    detel_calc2(&gimbal.angle_target);
    gimbal.speed_target = gimbal_PID_calc(&gimbal.pid_angle, gimbal.motor_info.real_angle, gimbal.angle_target);
}

// yaw轴控制电机
static void gimbal_yaw_control()
{
    if (mode_flag != 3)
    {
        gimbal.angle_target = 0;
    }

    gimbal.motor_info = motor_info_chassis[4];
    if (rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1] <= 660)
    {
        gimbal.angle_target += rc_ctrl.rc.ch[1] / 660.0 * 0.3;

        detel_calc(&gimbal.angle_target);

        err_yaw_angle = gimbal.angle_target - INS.Yaw;

        detel_calc(&err_yaw_angle);

        gimbal.speed_target = gimbal_PID_calc(&gimbal.pid_angle, 0, err_yaw_angle);
    }
}

static void detel_calc(fp32 *angle)
{
    if (*angle > 180)
    {
        *angle -= 360;
    }

    else if (*angle < -180)
    {
        *angle += 360;
    }
}

static void detel_calc2(fp32 *angle)
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
