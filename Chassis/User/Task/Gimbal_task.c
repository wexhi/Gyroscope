#include "Gimbal_task.h"
#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
extern motor_info_t motor_info_chassis[8]; // 电机信息结构体[3]为云台电机
fp32 gimbal_motor_pid[3] = {20, 0, 0};     // 云台电机的pid
pid_struct_t gimbal_pid_chassis;           // 云台电机的pid结构体
fp32 gimbal_speed_target = 100;            // 云台电机的目标速度

// 云台电机的初始化
static void Gimbal_loop_Init();

// 云台电机的任务
static void gimbal_current_give();

void Gimbal_task(void const *pvParameters)
{
    Gimbal_loop_Init();
    for (;;)
    {
        gimbal_current_give();
        osDelay(1);
    }
}

static void Gimbal_loop_Init()
{
    pid_init(&gimbal_pid_chassis, gimbal_motor_pid, 6000, 6000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
}

static void gimbal_current_give()
{
    motor_info_chassis[3].set_current = pid_calc(&gimbal_pid_chassis, motor_info_chassis[3].rotor_speed, gimbal_speed_target);
    set_motor_current_can1(0, 0, 0, 0, motor_info_chassis[3].set_current);
}