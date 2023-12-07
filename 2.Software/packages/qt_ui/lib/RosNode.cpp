#include "RosNode.h"

RosNode::RosNode(std::string name) : Node(name)
{
    //    subscripstion_ = this->create_subscription<std_msgs::msg::Float32>(
    //        "fishbot_battery_voltage", 10, std::bind(&RosNode::callback_subscription, this, std::placeholders::_1));
    subscripstion = this->create_subscription<std_msgs::msg::UInt8>(
        "fishbot_battery", 10, std::bind(&RosNode::callback_subscription, this, std::placeholders::_1));
}

void RosNode::callback_subscription(const std_msgs::msg::UInt8::SharedPtr msg)
{
    message_battry = msg->data;
    //    emit signal_fishbotBattery(msg->data);
}
