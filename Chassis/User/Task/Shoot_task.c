#include "Shoot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"

void Shoot_task(void const *pvParameters)
{
    for (;;)
    {
        osDelay(1);
    }
}