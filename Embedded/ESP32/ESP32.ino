/**
 * @author Avent
 * @date 2023-05-13
 */
#include "config/config.h"
#include "task_ros.hpp"
#include "tasks.hpp"

QueueHandle_t gMsgQueue = NULL;

inline void init()
{
    /*** Initialize I2C bus ***/
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    /** Initialize ADC (measure the voltage of the power supply) **/
    pinMode(ADC_PIN, INPUT);
    analogSetAttenuation(ADC_11db);
}
inline void setupCart()
{
    gCart.initMotor(MOTOR_L_PIN1, MOTOR_L_PIN2, MOTOR_R_PIN1, MOTOR_R_PIN2);
    gCart.initEncoder(ENCODER_L_PIN1, ENCODER_L_PIN2, ENCODER_R_PIN1, ENCODER_R_PIN2);
}

inline void setupROS()
{
    xTaskCreatePinnedToCore(task_ros, "Task MicroROS", 102400, NULL, 1, NULL, 0);
}

inline void setupTasks()
{
    xTaskCreatePinnedToCore(task_display, "Task Display", 2048, NULL, 1, NULL, 1);
}

inline void setupQueue()
{
    gMsgQueue = xQueueCreate(5, sizeof(uint8_t));
}

void setup()
{
    init();
    setupCart();
    setupROS();
    setupTasks();
    setupQueue();
}

void loop()
{
    /*** Do nothing. ***/
}