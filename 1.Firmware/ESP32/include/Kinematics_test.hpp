/**
 * @brief Calculate the forward and inverse kinematics of the robot
 * @file Kinematics.hpp
 * @author Avent
 * @date 2023-05-13
 */
#pragma once
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
// #include <Esp32PcntEncoder.h>

class Kinematics
{
public:
    explicit Kinematics();
    Kinematics(const Kinematics&)            = delete;   // 禁止拷贝构造函数
    Kinematics& operator=(const Kinematics&) = delete;   // 禁止赋值运算符
    ~Kinematics()                            = default;

    void forward(float wheel_speed_l, float wheel_speed_r);
    void inverse(float linear_speed, float angle_speed, float& wheel_speed_l, float& wheel_speed_r);

    static void euler2Quaternion(float roll, float pitch, float yaw, geometry_msgs__msg__Quaternion& q);
    static void transAngleInPi(float input, float& output);

    void updateOdom(uint32_t dt);

    // void getOdom(nav_msgs__msg__Odometry& odom_) { odom_ = this->odom; }
    nav_msgs__msg__Odometry odom;   // 里程计数据


private:
    const float wheel_distance = 72.e-3f;   // Distance between two wheels, unit: m
    float yaw;
};

Kinematics::Kinematics() : yaw(0.0)
{
    odom.header.frame_id.data = (char*)malloc(100 * sizeof(char));
    char string3[]            = "odom";
    memcpy(odom.header.frame_id.data, string3, strlen(string3) + 1);
    odom.header.frame_id.size     = strlen(odom.header.frame_id.data);
    odom.header.frame_id.capacity = 100;

    odom.child_frame_id.data = (char*)malloc(100 * sizeof(char));
    char string2[]           = "base_footprint";   // base_footprint
    memcpy(odom.child_frame_id.data, string2, strlen(string2) + 1);
    odom.child_frame_id.size     = strlen(odom.child_frame_id.data);
    odom.child_frame_id.capacity = 100;

    // robot's position in x,y, and z
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;

    // robot's heading in quaternion
    // odom.pose.pose.orientation.x = (double)q[1];
    // odom.pose.pose.orientation.y = (double)q[2];
    // odom.pose.pose.orientation.z = (double)q[3];
    // odom.pose.pose.orientation.w = (double)q[0];

    odom.pose.covariance[0]  = 0.001;
    odom.pose.covariance[7]  = 0.001;
    odom.pose.covariance[35] = 0.001;

    // linear speed from encoders
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    // angular speed from encoders
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom.twist.covariance[0]  = 0.0001;
    odom.twist.covariance[7]  = 0.0001;
    odom.twist.covariance[35] = 0.0001;
}

/*** 运动学正解，用于计算机器人的线速度和角速度 ***/
inline void Kinematics::forward(float wheel_l_speed, float wheel_r_speed)
{
    odom.twist.twist.linear.x  = (wheel_l_speed + wheel_r_speed) / 2.0f;
    odom.twist.twist.angular.z = (wheel_r_speed - wheel_l_speed) / wheel_distance;
}


/*** 运动学逆解，用于将机器人的线速度(m/s)和角速度(rad/s)转换为两个轮子的转速(m/s) ***/
inline void Kinematics::inverse(float linear_speed, float angular_speed, float& wheel_speed_l, float& wheel_speed_r)
{
    wheel_speed_l = linear_speed - (angular_speed * wheel_distance) / 2.0f;
    wheel_speed_r = linear_speed + (angular_speed * wheel_distance) / 2.0f;
}

/*** 用于将欧拉角转换为四元数 ***/
inline void Kinematics::euler2Quaternion(float roll, float pitch, float yaw, geometry_msgs__msg__Quaternion& q)
{
    // 传入机器人的欧拉角 roll、pitch 和 yaw。
    // 计算欧拉角的 sin 和 cos 值，分别保存在 cr、sr、cy、sy、cp、sp 六个变量中
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    /** 计算出四元数的四个分量 **/
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

/*** 用于将角度转换到 -π 到 π 的范围内 ***/
inline void Kinematics::transAngleInPi(float input, float& output)
{
    /** 如果 input 大于 π，则将 output 减去 2π **/
    if (input > PI)
    {
        output -= 2 * PI;
    }

    /** 如果 input 小于 -π，则将 output 加上 2π **/
    else if (input < -PI)
    {
        output += 2 * PI;
    }
}

// 机器人运动学模型的更新函数，用于更新机器人的位置和姿态信息
void Kinematics::updateOdom(uint32_t dt)
{
    float dt_s = (float)(dt / 1000.0f);
    // float dt_s = (float)(dt);

    // // 计算机器人的线速度和角速度。
    // this->forward(motor_param[0].motor_speed, motor_param[1].motor_speed, linear_speed, angular_speed);

    // 将计算出的角速度和线速度存储到 odom 结构体中。根据角速度和时间间隔，更新机器人的姿态信息 odom.yaw
    // odom.angular_speed = angular_speed;
    // odom.linear_speed  = linear_speed / 1000;   // /1000（mm/s 转 m/s）

    yaw += odom.twist.twist.angular.z * dt_s;
    // 将角度值 odom.yaw 转换到 -π 到 π 的范围内
    Kinematics::transAngleInPi(yaw, yaw);
    // Kinematics::transAngleInPi(odom.yaw, odom.yaw);
    euler2Quaternion(0, 0, yaw, odom.pose.pose.orientation);

    /*更新x和y轴上移动的距离*/
    // 计算机器人在 x 和 y 轴上移动的距离 delta_distance，单位为米
    float delta_distance = odom.twist.twist.linear.x * dt_s;   // 单位m

    // 根据机器人的角度值 odom.yaw 和移动距离 delta_distance，更新机器人的位置信息 odom.x 和 odom.y
    odom.pose.pose.position.x += delta_distance * std::cos(yaw);
    odom.pose.pose.position.y += delta_distance * std::sin(yaw);
}