#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

class RosNode : public rclcpp::Node
{
public:
    RosNode(std::string name) : Node(name)
    {
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "fishbot_odom", rclcpp::SensorDataQoS(), std::bind(&RosNode::odom_callback, this, std::placeholders::_1));
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    geometry_msgs::msg::TransformStamped transform;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double seconds            = this->now().seconds() + 1.0;
        transform.header.stamp    = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
        transform.header.frame_id = "odom";
        transform.child_frame_id  = "base_footprint";

        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation      = msg->pose.pose.orientation;
        tf_broadcaster->sendTransform(transform);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RosNode>("fishbot_bringup");
    /* 运行节点，并检测退出信号*/
    // rclcpp::WallRate loop_rate(1000.0);
    // while (rclcpp::ok())
    // {
    //     rclcpp::spin_some(node);
    //     node->publish_tf();
    //     loop_rate.sleep();
    // }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
