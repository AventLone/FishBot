#pragma once

namespace fishbot
{
typedef struct
{
    char* wifi_name;
    char* wifi_password;
    char* agent_ip;
    uint16_t agent_port;
} WifiConfig;

// typedef struct
// {
//     byte motor_l_pin1;
//     byte motor_l_pin2;
//     byte motor_r_pin1;
//     byte motor_r_pin2;

//     byte encoder_l_pin1;
//     byte encoder_l_pin2;
//     byte encoder_r_pin1;
//     byte encoder_r_pin2;
// } CartPins;


/*** 四元数 ***/
typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternion;

/*** 通用3D点 ***/
typedef struct
{
    float x;
    float y;
    float z;
} Vector3D;

/*** IMU数据 ***/
typedef struct
{
    Quaternion orientation;
    Vector3D angle_euler;
    Vector3D angular_velocity;
    Vector3D linear_acceleration;
} ImuData;


/**
 * @brief 电机相关结构体
 * @param  id   uint8_t  电机编号
 * @param  reducation_ratio uint16_t   减速器减速比，轮子转一圈，电机需要转的圈数
 * @param  pulse_ration uint16_t       脉冲比，电机转一圈所产生的脉冲数
 * @param  wheel_diameter float      轮子的外直径，单位mm
 * @param  per_pulse_distance float    无需配置，单个脉冲轮子前进的距离，单位mm，设置时自动计算
                               ,单个脉冲距离=轮子转一圈所行进的距离/轮子转一圈所产生的脉冲数
                               ,per_pulse_distance= (wheel_diameter*3.1415926)/(pulse_ration*reducation_ratio)
 * @param  speed_factor uint16_t 无需配置，计算速度时使用的速度因子，设置时自动计算，speed_factor计算方式如下
 *                             ,设 dt（单位us,1s=1000ms=10^6us）时间内的脉冲数为dtick
 *                             ,速度speed = per_pulse_distance*dtick/(dt/1000/1000)=(per_pulse_distance*1000*1000)*dtick/dt
 *                             ,记 speed_factor = (per_pulse_distance*1000*1000)
 * @param  motor_speed int16_t 无需配置，当前电机速度，计算时使用
 * @param  last_encoder_tick int64_t 无需配置，上次电机的编码器读数
 * @param last_update_time uint64_t 无需配置，上次更新数据的时间，单位us
*/

typedef struct
{
    uint8_t id;                  // 电机编号
    uint16_t reducation_ratio;   // 减速器减速比，轮子转一圈，电机需要转的圈数
    uint16_t pulse_ration;       // 脉冲比，电机转一圈所产生的脉冲数
    float wheel_diameter;        // 轮子的外直径，单位mm

    float per_pulse_distance;    // 无需配置，单个脉冲轮子前进的距离，单位mm，设置时自动计算
                                 // 单个脉冲距离=轮子转一圈所行进的距离/轮子转一圈所产生的脉冲数
                                 // per_pulse_distance= (wheel_diameter*3.1415926)/(pulse_ration*reducation_ratio)
    uint32_t speed_factor;   // 无需配置，计算速度时使用的速度因子，设置时自动计算，speed_factor计算方式如下
                             // 设 dt（单位us,1s=1000ms=10^6us）时间内的脉冲数为dtick
                             // 速度speed = per_pulse_distance*dtick/(dt/1000/1000)=(per_pulse_distance*1000*1000)*dtic/dt
                             // 记 speed_factor = (per_pulse_distance*1000*1000)
    int16_t motor_speed;         // 无需配置，当前电机速度mm/s，计算时使用
    int64_t last_encoder_tick;   // 无需配置，上次电机的编码器读数
    uint64_t last_update_time;   // 无需配置，上次更新数据的时间，单位us
} MotorParam;

/*** 里程计 ***/
typedef struct
{
    float x;                 // 坐标x
    float y;                 // 坐标y
    float yaw;               // yawl
    Quaternion quaternion;   // 姿态四元数
    float linear_speed;
    float angular_speed;
} Odometer;
}