/**
 * @brief Calculate the forward and inverse kinematics of the robot
 * @file Kinematics.hpp
 * @author Avent
 * @date 2023-05-13
 */
#pragma once
// #include <nav_msgs/msg/odometry.h>

class Kinematics
{
public:
    Kinematics() = default;

    void forward(float wheel_speed_l, float wheel_speed_r);
    void inverse(float linear_speed, float angle_speed);

public:
    float linear_speed, angular_speed;
    float wheel_speed_l, wheel_speed_r;


private:
    const float wheel_distance = 72.e-3f;   // Distance between two wheels, unit: m
};

inline void Kinematics::forward(float wheel_speed_l_, float wheel_speed_r_)
{
    linear_speed  = (wheel_speed_l_ + wheel_speed_r_) / 2.0f;
    angular_speed = (wheel_speed_r_ - wheel_speed_l_) / wheel_distance;
}


/*** 运动学逆解，用于将机器人的线速度(m/s)和角速度(rad/s)转换为两个轮子的转速(m/s) ***/
inline void Kinematics::inverse(float linear_speed_, float angular_speed_)
{
    wheel_speed_l = linear_speed_ - (angular_speed * wheel_distance) / 2.0f;
    wheel_speed_r = linear_speed_ + (angular_speed * wheel_distance) / 2.0f;
}