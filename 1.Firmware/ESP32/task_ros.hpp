/**
 * @author Avent
 * @date 2023-05-13
 */
#pragma once
#include "include/MiroRosNode.hpp"
#include "include/Cart.hpp"
#include "include/Kinematics.hpp"
// #include "include/Kinematics_test.hpp"
#include <std_msgs/msg/u_int8.h>
#include <geometry_msgs/msg/twist.h>
// #include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>

const fishbot::WifiConfig gWifiConfig = {"AventPhone", "qwertyuiop", "172.20.10.2", 8888};

builtin_interfaces__msg__Time gTimeStamp;   // Used to get the current time stamp

rcl_publisher_t gBatteryPublisher;
std_msgs__msg__UInt8 gBatteryMsg;

// rcl_publisher_t odom_publisher;
// nav_msgs__msg__Odometry odom_msg;

// rcl_publisher_t joint_states_publisher;
// sensor_msgs__msg__JointState joint_states_msg;
// void init_joint_states_msg();

/*** Subscription callback ***/
void twistCallback(const void* msg);
/*** Timer callback ***/
void publishBatteryInfo(rcl_timer_t* timer, int64_t last_call_time);
void callback_timer_joint_states(rcl_timer_t* timer, int64_t last_call_time);
void callback_timer_odom(rcl_timer_t* timer, int64_t last_call_time);

void task_ros(void* param)
{
    MicroRosNode ros_node("fishbot_esp32", 2, gWifiConfig);

    rcl_subscription_t subscriber;
    geometry_msgs__msg__Twist sub_msg;

    // rcl_timer_t battery_timer, timer_joint_states, timer_odom;
    rcl_timer_t battery_timer;

    // init_joint_states_msg();

    ros_node.addSubscription(subscriber,
                             ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                             "cmd_vel",
                             &sub_msg,
                             twistCallback,
                             ON_NEW_DATA);

    ros_node.addTimer(battery_timer, 1000, publishBatteryInfo);
    // ros_node.add_timer(timer_joint_states, 50, callback_timer_odom);
    // ros_node.add_timer(timer_odom, 500, callback_timer_joint_states);
    ros_node.addPublisher(gBatteryPublisher, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "fishbot_battery");
    // ros_node.add_publisher(odom_publisher, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "fishbot_odom");
    // ros_node.add_publisher(
    //     joint_states_publisher, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "fishbot_joint_states");

    ros_node.syncTime();

    ros_node.spin();
}

/*** Process the Twist message from the host computer ***/
Cart gCart;
Kinematics gKinematics;
void twistCallback(const void* msg)
{
    /*** 将接收到的消息指针转化为 geometry_msgs__msg__Twist 类型 ***/
    const geometry_msgs__msg__Twist* twist_msg = (const geometry_msgs__msg__Twist*)msg;

    float linear_x = twist_msg->linear.x;
    float angular_z = twist_msg->angular.z;

    if (linear_x == 0 && angular_z == 0)
    {
        gCart.motorSpin(MOTOR_L, 0);
        gCart.motorSpin(MOTOR_R, 0);
        return;
    }
    if (linear_x > 0)
    {
        gCart.goForward(100);
    }
    if (linear_x < 0)
    {
        gCart.goBackward(100);
    }
    if (angular_z > 0)
    {
        gCart.turnLeft(100);
    }
    if (angular_z < 0)
    {
        gCart.turnRight(100);
    }
}

/** Send battery information at regular intervals **/
extern QueueHandle_t gMsgQueue;
void publishBatteryInfo(rcl_timer_t* timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        xQueueReceive(gMsgQueue, &gBatteryMsg.data, 0);
        rcl_publish(&gBatteryPublisher, &gBatteryMsg, NULL);
    }
}

// void twistCallback(const void* msg)
// {
//     /*** 将接收到的消息指针转化为 geometry_msgs__msg__Twist 类型 ***/
//     const geometry_msgs__msg__Twist* twist_msg = (const geometry_msgs__msg__Twist*)msg;

//     float linear_x  = twist_msg->linear.x;
//     float angular_z = twist_msg->angular.z;

//     if (linear_x == 0 && angular_z == 0)
//     {
//         gCart.motorSpin(MOTOR_L, 0);
//         gCart.motorSpin(MOTOR_R, 0);
//         return;
//     }
//     gKinematics.inverse(linear_x, angular_z);
//     gCart.leftMotorSpin(gKinematics.wheel_speed_l);
//     gCart.rightMotorSpin(gKinematics.wheel_speed_r);
//     // gCart.motorSpin(MOTOR_L, gKinematics.motor_speeds[0]);
// }


/*** Send robot state information at regular intervals ***/
// void callback_timer_odom(rcl_timer_t* timer, int64_t last_call_time)
// {
//     RCLC_UNUSED(last_call_time);
//     if (timer != NULL)
//     {
//         /*** Publish odometry message ***/
//         gCart.getMotorsSpeed();
//         gKinematics.forward(gCart.motor_speeds[0], gCart.motor_speeds[1]);
//         gKinematics.updateOdom(gCart.dt);

//         /** Get the current time stamp and store it in the header of the message **/
//         int64_t now_millis = rmw_uros_epoch_millis();
//         gTimeStamp.sec     = now_millis / 1000;
//         gTimeStamp.nanosec = (now_millis % 1000) * 1000000;

//         gKinematics.odom.header.stamp = gTimeStamp;
//         rcl_publish(&odom_publisher, &gKinematics.odom, NULL);


//         /*** Publish joint state message ***/
//         // gCart.getAngularPosition(joint_states_msg.position.data);
//         // joint_states_msg.header.stamp      = gTimeStamp;
//         // joint_states_msg.name.data[0].data = "leftwheel_joint";
//         // joint_states_msg.name.data[1].data = "rightwheel_joint";
//         // gCart.getAngularPosition(joint_states_msg.position.data);
//         // rcl_publish(&joint_states_publisher, &joint_states_msg, NULL);
//     }
// }

/*** Publish joint state message ***/
// void callback_timer_joint_states(rcl_timer_t* timer, int64_t last_call_time)
// {
//     RCLC_UNUSED(last_call_time);
//     if (timer != NULL)
//     {
//         int64_t now_millis = rmw_uros_epoch_millis();
//         gTimeStamp.sec     = now_millis / 1000;
//         gTimeStamp.nanosec = (now_millis % 1000) * 1000000;

//         joint_states_msg.header.stamp      = gTimeStamp;
//         joint_states_msg.name.data[0].data = "leftwheel_joint";
//         joint_states_msg.name.data[1].data = "rightwheel_joint";
//         gCart.getAngularPosition(joint_states_msg.position.data);
//         rcl_publish(&joint_states_publisher, &joint_states_msg, NULL);
//     }
// }

// void init_joint_states_msg()
// {
// #define JOINT_NUM 2

//     char msg_joint_state_buffer[10];
//     joint_states_msg.header.frame_id.data     = msg_joint_state_buffer;
//     joint_states_msg.header.frame_id.size     = 1;
//     joint_states_msg.header.frame_id.capacity = 1;

//     rosidl_runtime_c__String string_buffer[2];
//     joint_states_msg.name.data     = string_buffer;
//     joint_states_msg.name.size     = 2;
//     joint_states_msg.name.capacity = 18 * 2;

//     for (int i = 0; i < JOINT_NUM; i++)
//     {
//         joint_states_msg.name.data[i].data     = (char*)malloc(sizeof(char) * 18);
//         joint_states_msg.name.data[i].size     = 1;
//         joint_states_msg.name.data[i].capacity = 18;
//     }

//     double joint_states_position_buffer[2];
//     joint_states_msg.position.data     = joint_states_position_buffer;
//     joint_states_msg.position.size     = JOINT_NUM;
//     joint_states_msg.position.capacity = sizeof(double) * JOINT_NUM;

//     // double joint_states_velocity_buffer[2];
//     // joint_states_msg.velocity.data     = joint_states_velocity_buffer;
//     // joint_states_msg.velocity.size     = JOINT_NUM;
//     // joint_states_msg.velocity.capacity = sizeof(double) * JOINT_NUM;

//     // double joint_states_effort_buffer[JOINT_DOUBLE_LEN];
//     // joint_states_msg.effort.data     = joint_states_effort_buffer;
//     // joint_states_msg.effort.size     = 2;
//     // joint_states_msg.effort.capacity = sizeof(double);
// }