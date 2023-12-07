/**
 * @brief Get the pose of the robot
 * @file Imu.hpp
 * @author Avent
 * @date 2023-05-16
 */
#pragma once
#include <MPU6050_light.h>

class Imu
{
public:
    /** 同u哦NPU6050构造一个新的IMU对象 **/
    Imu()                      = default;
    Imu(const Imu&)            = delete;   // 禁止拷贝构造函数
    Imu& operator=(const Imu&) = delete;   // 禁止赋值运算符
    ~Imu()                     = default;

    /**
     * @brief 初始化函数
     * @param sda 引脚编号
     * @param scl 引脚编号
     * @return true
     * @return false
     */
    bool begin();

    /**
     * @brief 欧拉角转四元数
     * @param roll 输入X
     * @param pitch 输入y
     * @param yaw 输入Z
     * @param q  返回的四元数引用
     */
    static void euler2Quaternion(float roll, float pitch, float yaw, Quaternion& q);

    /** 获取IMU数据函数 **/
    void getImuData(ImuData& imu_data);

    /**
     * @brief 更新IMU数据，同上一节中的mou.update
     */
    void update();

private:
    MPU6050 mpu = MPU6050(Wire);   // mpu6050指针
};

bool Imu::begin()
{
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    if (status != 0)
    {
        for (uint8_t i = 0; i < 10; i++)
        {
            if (status == 0) break;
            delay(200);
        }
        if (status != 0)
        {
            return false;
        }
    }   // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
    mpu.calcOffsets();   // gyro and accelero
    Serial.println("Done!\n");
    return true;
}

void Imu::euler2Quaternion(float roll, float pitch, float yaw, Quaternion& q)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

void Imu::getImuData(ImuData& imu_data)
{
    // imu_data.angle_euler.x = mpu->getAngleX();
    // imu_data.angle_euler.y = -mpu->getAngleY();
    // imu_data.angle_euler.z = mpu->getAngleZ();

    // imu_data.angular_velocity.x = mpu_->getAccAngleX();
    // imu_data.angular_velocity.y = -mpu_->getAccAngleY();
    // imu_data.angular_velocity.z = mpu_->getGyroZ();

    // imu_data.linear_acceleration.x = mpu_->getAccX();
    // imu_data.linear_acceleration.y = mpu_->getAccY();
    // imu_data.linear_acceleration.z = mpu_->getAccZ();
    imu_data.angle_euler.x = mpu.getAngleX();
    imu_data.angle_euler.y = -mpu.getAngleY();
    imu_data.angle_euler.z = mpu.getAngleZ();

    imu_data.angular_velocity.x = mpu.getAccAngleX();
    imu_data.angular_velocity.y = -mpu.getAccAngleY();
    imu_data.angular_velocity.z = mpu.getGyroZ();

    imu_data.linear_acceleration.x = mpu.getAccX();
    imu_data.linear_acceleration.y = mpu.getAccY();
    imu_data.linear_acceleration.z = mpu.getAccZ();

    Imu::euler2Quaternion(imu_data.angle_euler.x, imu_data.angle_euler.y, imu_data.angle_euler.z, imu_data.orientation);
}

inline void Imu::update()
{
    mpu.update();
}
