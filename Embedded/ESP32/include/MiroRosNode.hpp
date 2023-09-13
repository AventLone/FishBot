/**
 * @brief Micro-ROS Node class
 * @file MicroRosNode.hpp
 * @author Avent
 * @date 2023-05-13
 */
#pragma once
#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <TimeLib.h>

class MicroRosNode
{
public:
    explicit MicroRosNode(const char* node_name, size_t handles_num);
    explicit MicroRosNode(const char* node_name, size_t handles_num, const fishbot::WifiConfig& wifi_config);
    MicroRosNode(const MicroRosNode&) = delete;              // Disable copy constructor
    MicroRosNode& operator=(const MicroRosNode&) = delete;   // Disable assignment operator
    ~MicroRosNode() = default;

    void spin()
    {
        rclc_executor_spin(&executor);
    }
    void spinOnce()
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    void addSubscription(rcl_subscription_t& subscription, const rosidl_message_type_support_t* type_support,
                         const char* topic_name, void* msg, rclc_subscription_callback_t callback,
                         rclc_executor_handle_invocation_t invocation)
    {
        rclc_subscription_init_default(&subscription, &node, type_support, topic_name);
        rclc_executor_add_subscription(&executor, &subscription, msg, callback, invocation);
    }

    void addTimer(rcl_timer_t& timer, uint64_t period, rcl_timer_callback_t callback)
    {
        rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(period), callback);
        rclc_executor_add_timer(&executor, &timer);
    }

    void addPublisher(rcl_publisher_t& publisher, const rosidl_message_type_support_t* type_support,
                      const char* topic_name)
    {
        rclc_publisher_init_default(&publisher, &node, type_support, topic_name);
    }

    void syncTime();

private:
    rclc_executor_t executor;    // 创建一个 RCLC 执行程序对象，用于处理订阅和发布
    rclc_support_t support;      // 创建一个 RCLC 支持对象，用于管理 ROS2 上下文和节点
    rcl_allocator_t allocator;   // 创建一个 RCL 分配器对象，用于分配内存
    rcl_node_t node;             // 创建一个 RCL 节点对象
};

MicroRosNode::MicroRosNode(const char* node_name, size_t handles_num)
{
    Serial.begin(115200);
    set_microros_transports();
    vTaskDelay(1000 / portTICK_PERIOD_MS);   // Wait for setting done

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, node_name, "", &support);

    rclc_executor_init(&executor, &support.context, handles_num, &allocator);
}

MicroRosNode::MicroRosNode(const char* node_name, size_t handles_num, const fishbot::WifiConfig& wifi_config)
{
    set_microros_wifi_transports(
        wifi_config.wifi_name, wifi_config.wifi_password, wifi_config.agent_ip, wifi_config.agent_port);
    vTaskDelay(1500 / portTICK_PERIOD_MS);   // Wait for setting done

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, node_name, "", &support);

    rclc_executor_init(&executor, &support.context, handles_num, &allocator);
}

void MicroRosNode::syncTime()
{
    const int timeout_ms = 1000;
    int64_t time_ms;
    time_t time_seconds;
    while (!rmw_uros_epoch_synchronized())   // 判断时间是否同步
    {
        rmw_uros_sync_session(timeout_ms);   //  同步时间
        if (rmw_uros_epoch_synchronized())
        {
            time_ms = rmw_uros_epoch_millis();   // 获取当前时间
            time_seconds = time_ms / 1000;
            setTime(time_seconds + 8 * 3600);   // 将当前时间+8H到北京时间然后设置到系统
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}