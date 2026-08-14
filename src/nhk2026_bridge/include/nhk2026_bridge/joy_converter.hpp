#ifndef JOY_CONVERTER_HPP_
#define JOY_CONVERTER_HPP_

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

# include <bits/stdc++.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class JoyConverter
: public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit JoyConverter();

private:
    // Lifecycle callbacks
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    // Joy callback
    void joy_callback(
        const sensor_msgs::msg::Joy::SharedPtr msg);

    // Parameter callback
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters);

    // Publish zero velocity
    void publish_zero_velocity();

    // Get axis value safely
    double get_axis(
        const sensor_msgs::msg::Joy::SharedPtr msg,
        int index) const;

    // Apply deadzone
    double apply_deadzone(double value) const;


private:
    // Publisher / Subscriber
    rclcpp_lifecycle::LifecyclePublisher<
        geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    // Parameter callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::
        SharedPtr parameter_callback_handle_;

    // Topic names
    std::string input_topic_;
    std::string output_topic_;

    // Maximum velocities
    double max_vx_;
    double max_vy_;
    double max_omega_;

    // Joystick axis assignment
    int axis_linear_x_;
    int axis_linear_y_;
    int axis_angular_z_;

    // Sign inversion
    bool invert_linear_x_;
    bool invert_linear_y_;
    bool invert_angular_z_;

    // Deadzone
    double deadzone_;

    // Joy timeout
    double joy_timeout_;

    // Last Joy reception time
    rclcpp::Time last_joy_time_;
};

#endif  // JOY_CONVERTER_HPP_