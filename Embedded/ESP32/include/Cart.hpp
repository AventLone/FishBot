/**
 * @brief Control the cart
 * @file Cart.hpp
 * @author Avent
 * @date 2023-05-13
 */
#pragma once
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
// #include "PidController.hpp"


class Cart
{
public:
    // explicit Cart(byte motor_l_pin1, byte motor_l_pin2, byte motor_r_pin1, byte motor_r_pin2);
    // explicit Cart(const fishbot::CartPins& cart_pins);
    explicit Cart()              = default;
    Cart(const Cart&)            = delete;   // 禁止拷贝构造函数
    Cart& operator=(const Cart&) = delete;   // 禁止赋值运算符
    ~Cart()                      = default;

    /*** Initialize motors and encoders ***/
    void initMotor(byte motor_l_pin1, byte motor_l_pin2, byte motor_r_pin1, byte motor_r_pin2);
    void initEncoder(byte encoder_l_pin1, byte encoder_l_pin2, byte encoder_r_pin1, byte encoder_r_pin2);

    /*** Make motors spin ***/
    void motorSpin(int8_t motor_id, int16_t speed);
    void leftMotorSpin(float speed);
    void rightMotorSpin(float speed);

    /*** Control the movement of the cart ***/
    void goForward(int speed);
    void goBackward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);

    /*** Get the states of the cart ***/
    void getMotorsSpeed();
    void getAngularPosition(double angular_position[]);

public:
    float motor_speeds[2];   // 记录两个电机的转速
    uint64_t dt;             // 记录两次读取之间的时间差

private:
    Esp32McpwmMotor motor;                   // Construct a object named "motor" to control the motor
    const float wheel_diameter = 67.2e-3f;   // Diameter of the wheel, unit: m
    // float wheel_distance;   // 轮子间距，单位m

    Esp32PcntEncoder encoders[2];   // Create an array to store two encoders

    int64_t last_ticks[2];          // 记录上一次读取的计数器数值
    int32_t pt[2];                  // 记录两次读取之间的计数器差值
    int64_t last_update_time = 0;   // 记录上一次更新时间
};

inline void Cart::initMotor(byte motor_l_pin1, byte motor_l_pin2, byte motor_r_pin1, byte motor_r_pin2)
{
    pinMode(motor_l_pin1, OUTPUT);
    pinMode(motor_l_pin2, OUTPUT);
    pinMode(motor_r_pin1, OUTPUT);
    pinMode(motor_r_pin2, OUTPUT);

    motor.attachMotor(MOTOR_L, motor_l_pin1, motor_l_pin2);
    motor.attachMotor(MOTOR_R, motor_r_pin1, motor_r_pin2);
}

inline void Cart::initEncoder(byte encoder_l_pin1, byte encoder_l_pin2, byte encoder_r_pin1, byte encoder_r_pin2)
{
    encoders[ENCODER_L].init(ENCODER_L, encoder_l_pin1, encoder_l_pin2);
    encoders[ENCODER_R].init(ENCODER_R, encoder_r_pin1, encoder_r_pin2);
}

inline void Cart::motorSpin(int8_t motor_id, int16_t speed)
{
    motor.updateMotorSpeed(motor_id, speed);
}

inline void Cart::leftMotorSpin(float speed)
{
    /*** Remap ***/
    int16_t speed_remaped = int16_t(speed * 32.0f / max_linear_speed_l) + 68;
    motor.updateMotorSpeed(MOTOR_L, -speed_remaped);
}
inline void Cart::rightMotorSpin(float speed)
{
    int16_t speed_remaped = int16_t(speed * 25.0f / max_linear_speed_r) + 75;
    motor.updateMotorSpeed(MOTOR_R, speed_remaped);
}

/*** Cart go forward and backward ***/
inline void Cart::goForward(int speed)
{
    motor.updateMotorSpeed(MOTOR_L, -speed);
    motor.updateMotorSpeed(MOTOR_R, speed);
}
inline void Cart::goBackward(int speed)
{
    motor.updateMotorSpeed(MOTOR_L, speed);
    motor.updateMotorSpeed(MOTOR_R, -speed);
}


/*** Cart turn left or right ***/
inline void Cart::turnLeft(int speed)
{
    motor.updateMotorSpeed(MOTOR_L, speed);
    motor.updateMotorSpeed(MOTOR_R, speed);
}
inline void Cart::turnRight(int speed)
{
    motor.updateMotorSpeed(MOTOR_L, -speed);
    motor.updateMotorSpeed(MOTOR_R, -speed);
}


/*** Get the speed of the 2 motors (转/s) ***/
inline void Cart::getMotorsSpeed()
{
    // 计算两个电机的速度
    dt = millis() - last_update_time;                                         // 计算两次读取之间的时间差
    pt[ENCODER_L] = encoders[ENCODER_L].getTicks() - last_ticks[ENCODER_L];   // 计算左编码器两次读取之间的计数器差值
    pt[ENCODER_R] = encoders[ENCODER_R].getTicks() - last_ticks[ENCODER_R];   // 计算右编码器两次读取之间的计数器差值
    motor_speeds[ENCODER_L] = PI * wheel_diameter * float(pt[ENCODER_L] * 1000.0f) / float(dt * PPR_L);   // 计算左电机的速度
    motor_speeds[ENCODER_R] = PI * wheel_diameter * float(pt[ENCODER_R] * 1000.0f) / float(dt * PPR_R);   // 计算右电机的速度

    // 更新记录
    last_update_time      = millis();                         // 更新上一次更新时间
    last_ticks[ENCODER_L] = encoders[ENCODER_L].getTicks();   // 更新左编码器的计数器数值
    last_ticks[ENCODER_R] = encoders[ENCODER_R].getTicks();   // 更新右编码器的计数器数值
}

/*** Get the angular position of the 2 motors ***/
inline void Cart::getAngularPosition(double angular_position[])
{
    angular_position[0] = double(encoders[ENCODER_L].getTicks() % PPR_L) / PPR_L * 2 * PI;
    /** 如果 input 大于 π，则将 output 减去 2π **/
    if (angular_position[0] > PI)
    {
        angular_position[0] -= 2 * PI;
    }
    /** 如果 input 小于 -π，则将 output 加上 2π **/
    else if (angular_position[0] < -PI)
    {
        angular_position[0] += 2 * PI;
    }
    angular_position[1] = double(encoders[ENCODER_R].getTicks() % PPR_R) / PPR_R * 2 * PI;
    /** 如果 input 大于 π，则将 output 减去 2π **/
    if (angular_position[1] > PI)
    {
        angular_position[1] -= 2 * PI;
    }
    /** 如果 input 小于 -π，则将 output 加上 2π **/
    else if (angular_position[1] < -PI)
    {
        angular_position[1] += 2 * PI;
    }
    angular_position[1] = -angular_position[1];
}
