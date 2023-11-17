#include "Shoot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"

shooter_t shooter; // 发射机构信息结构体
// 电机0为拨盘电机，电机1为弹舱盖电机，电机2、3为摩擦轮电机

void Shoot_task(void const *pvParameters)
{
    for (;;)
    {
        osDelay(1);
    }
}