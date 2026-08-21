#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class JoyChanger : public rclcpp_lifecycle::LifecycleNode
{
public:
    JoyChanger();
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy);
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::ByteMultiArray>::SharedPtr
        air_cylinder_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32MultiArray>::SharedPtr
        robomasu_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32MultiArray>::SharedPtr
        ejection_publisher_;

    int32_t maxhand_rotation_;
    int32_t minhand_rotation_;
    double hand_rotation_speed_;
    double hand_rotation_;
    rclcpp::Time last_joy_stamp_;
    bool has_last_joy_stamp_;
    int32_t maxball_holder_;
    int32_t max_ejectionrpm_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};
