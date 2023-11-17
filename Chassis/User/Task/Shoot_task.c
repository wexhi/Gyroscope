#include "Shoot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"

shooter_t shooter; // 发射机构信息结构体
// 电机0为拨盘电机，电机1为弹舱盖电机，电机2、3为摩擦轮电机

static void Shooter_Inint(); // 发射机构的初始化

void Shoot_task(void const *pvParameters)
{
    Shooter_Inint();
    for (;;)
    {
        osDelay(1);
    }
}

// 发射机构的初始化
static void Shooter_Inint(void)
{
    // 初始化pid参数
    shooter.pid_dial_para[0] = 20, shooter.pid_dial_para[1] = 0, shooter.pid_dial_para[2] = 0;
    shooter.pid_friction_para[0] = 30, shooter.pid_friction_para[1] = 0, shooter.pid_friction_para[2] = 0;
    shooter.pid_bay_para[0] = 10, shooter.pid_bay_para[1] = 0, shooter.pid_bay_para[2] = 0;

    // 初始化pid结构体
    pid_init(&shooter.pid_dial, shooter.pid_dial_para, 16384, 16384);
    pid_init(&shooter.pid_friction, shooter.pid_friction_para, 16384, 16384);
    pid_init(&shooter.pid_bay, shooter.pid_bay_para, 16384, 16384);

    // 初始化速度目标
    shooter.dial_speed_target = 0;
    shooter.friction_speed_target = 0;
    shooter.bay_speed_target = 0;
}