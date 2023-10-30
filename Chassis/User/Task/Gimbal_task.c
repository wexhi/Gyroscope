#include "Gimbal_task.h"
#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"

void Gimbal_task(void const *pvParameters)
{
    for (;;)
    {
        osDelay(1);
    }
}

static void gimbal_current_give()
{
}