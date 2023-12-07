#pragma once
#include "include/Screen.hpp"

extern QueueHandle_t gMsgQueue;

/*** 使用宏来达到只初始化一次的效果是错误的！ ***/
/** inline void initI2C()
{
#ifndef WIRE_I2C_INIT
#define WIRE_I2C_INIT
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);   // 初始化I2C总线
#endif
}**/

void task_display(void* param)
{
    Screen screen(Wire);
    for (;;)
    {
        screen.showBatteryPower();
        xQueueSend(gMsgQueue, &screen.battery_power, 10);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
