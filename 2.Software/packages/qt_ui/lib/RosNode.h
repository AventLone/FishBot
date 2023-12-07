#ifndef __ROSNODE_H
#define __ROSNODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

class RosNode : public rclcpp::Node
{
public:
    /*** Constructor & Destructor ***/
    RosNode(std::string name);

    /*** Public member variable ***/
    //    float volts = 0.0;
    uint8_t message_battry = 100;


private:
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscripstion = nullptr;

    void callback_subscription(const std_msgs::msg::UInt8::SharedPtr msg);
};

#endif
