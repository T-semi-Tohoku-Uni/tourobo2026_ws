#include "joy_converter.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

using std::placeholders::_1;

JoyConverter::JoyConverter()
: rclcpp_lifecycle::LifecycleNode("joy_converter")
{
    // Topic
    this->declare_parameter<std::string>(
        "input_topic", "joy");

    this->declare_parameter<std::string>(
        "output_topic", "cmd_vel");


    // Maximum velocity
    this->declare_parameter<double>(
        "max_vx", 2.0);

    this->declare_parameter<double>(
        "max_vy", 2.0);

    this->declare_parameter<double>(
        "max_omega", M_PI);


    // Axis assignment
    this->declare_parameter<int>(
        "axis_linear_x", 0);

    this->declare_parameter<int>(
        "axis_linear_y", 1);

    this->declare_parameter<int>(
        "axis_angular_z", 3);


    // Sign inversion
    this->declare_parameter<bool>(
        "invert_linear_x", false);

    this->declare_parameter<bool>(
        "invert_linear_y", false);

    this->declare_parameter<bool>(
        "invert_angular_z", false);


    // Deadzone
    this->declare_parameter<double>(
        "deadzone", 0.05);


    // Timeout
    this->declare_parameter<double>(
        "joy_timeout", 0.1);


    // Read parameters
    this->input_topic_ =
        this->get_parameter("input_topic").as_string();

    this->output_topic_ =
        this->get_parameter("output_topic").as_string();


    this->max_vx_ =
        this->get_parameter("max_vx").as_double();

    this->max_vy_ =
        this->get_parameter("max_vy").as_double();

    this->max_omega_ =
        this->get_parameter("max_omega").as_double();


    this->axis_linear_x_ =
        this->get_parameter("axis_linear_x").as_int();

    this->axis_linear_y_ =
        this->get_parameter("axis_linear_y").as_int();

    this->axis_angular_z_ =
        this->get_parameter("axis_angular_z").as_int();


    this->invert_linear_x_ =
        this->get_parameter("invert_linear_x").as_bool();

    this->invert_linear_y_ =
        this->get_parameter("invert_linear_y").as_bool();

    this->invert_angular_z_ =
        this->get_parameter("invert_angular_z").as_bool();


    this->deadzone_ =
        this->get_parameter("deadzone").as_double();

    this->joy_timeout_ =
        this->get_parameter("joy_timeout").as_double();


    // Dynamic parameter callback
    this->parameter_callback_handle_ =
        this->add_on_set_parameters_callback(
            std::bind(
                &JoyConverter::parameters_callback,
                this,
                std::placeholders::_1
            )
        );
}


JoyConverter::CallbackReturn
JoyConverter::on_configure(
    const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Configuring joy_converter...");


    // Publisher
    this->cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(
            this->output_topic_,
            rclcpp::SystemDefaultsQoS()
        );


    // Subscriber
    this->joy_subscriber_ =
        this->create_subscription<sensor_msgs::msg::Joy>(
            this->input_topic_,
            rclcpp::SystemDefaultsQoS(),
            std::bind(
                &JoyConverter::joy_callback,
                this,
                _1
            )
        );

    this->last_joy_time_ = this->now();

    RCLCPP_INFO(
        this->get_logger(),
        "input topic  : %s",
        this->input_topic_.c_str());

    RCLCPP_INFO(
        this->get_logger(),
        "output topic : %s",
        this->output_topic_.c_str());

    RCLCPP_INFO(
        this->get_logger(),
        "axis X       : %d",
        this->axis_linear_x_);

    RCLCPP_INFO(
        this->get_logger(),
        "axis Y       : %d",
        this->axis_linear_y_);

    RCLCPP_INFO(
        this->get_logger(),
        "axis omega   : %d",
        this->axis_angular_z_);


    RCLCPP_INFO(
        this->get_logger(),
        "on_configure() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());


    return CallbackReturn::SUCCESS;
}


JoyConverter::CallbackReturn
JoyConverter::on_activate(
    const rclcpp_lifecycle::State & state)
{
    this->cmd_vel_publisher_->on_activate();


    RCLCPP_INFO(
        this->get_logger(),
        "joy_converter activated");


    RCLCPP_INFO(
        this->get_logger(),
        "on_activate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());


    return CallbackReturn::SUCCESS;
}


JoyConverter::CallbackReturn
JoyConverter::on_deactivate(
    const rclcpp_lifecycle::State & state)
{
    this->publish_zero_velocity();


    if (this->cmd_vel_publisher_) {
        this->cmd_vel_publisher_->on_deactivate();
    }

    RCLCPP_INFO(
        this->get_logger(),
        "joy_converter deactivated");

    RCLCPP_INFO(
        this->get_logger(),
        "on_deactivate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());

    return CallbackReturn::SUCCESS;
}


JoyConverter::CallbackReturn
JoyConverter::on_cleanup(
    const rclcpp_lifecycle::State & state)
{
    this->joy_subscriber_.reset();
    this->cmd_vel_publisher_.reset();


    RCLCPP_INFO(
        this->get_logger(),
        "on_cleanup() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());


    return CallbackReturn::SUCCESS;
}


JoyConverter::CallbackReturn
JoyConverter::on_error(
    const rclcpp_lifecycle::State & state)
{
    this->publish_zero_velocity();


    RCLCPP_INFO(
        this->get_logger(),
        "on_error() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());


    return CallbackReturn::SUCCESS;
}


JoyConverter::CallbackReturn
JoyConverter::on_shutdown(
    const rclcpp_lifecycle::State & state)
{
    this->publish_zero_velocity();


    this->joy_subscriber_.reset();
    this->cmd_vel_publisher_.reset();


    RCLCPP_INFO(
        this->get_logger(),
        "on_shutdown() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());


    return CallbackReturn::SUCCESS;
}


void JoyConverter::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (!this->cmd_vel_publisher_) {
        return;
    }

    if (!this->cmd_vel_publisher_->is_activated()) {
        return;
    }


    // Update last reception time
    this->last_joy_time_ = this->now();


    geometry_msgs::msg::Twist cmd_vel;


    // Get joystick values
    double x =
        this->apply_deadzone(
            this->get_axis(msg, this->axis_linear_x_)
        );

    double y =
        this->apply_deadzone(
            this->get_axis(msg, this->axis_linear_y_)
        );

    double omega =
        this->apply_deadzone(
            this->get_axis(msg, this->axis_angular_z_)
        );


    // Sign inversion
    if (this->invert_linear_x_) {
        x = -x;
    }

    if (this->invert_linear_y_) {
        y = -y;
    }

    if (this->invert_angular_z_) {
        omega = -omega;
    }


    // Convert to velocity
    cmd_vel.linear.x =
        x * this->max_vx_;

    cmd_vel.linear.y =
        y * this->max_vy_;

    cmd_vel.linear.z =
        0.0;


    cmd_vel.angular.x =
        0.0;

    cmd_vel.angular.y =
        0.0;

    cmd_vel.angular.z =
        omega * this->max_omega_;


    // Publish
    this->cmd_vel_publisher_->publish(cmd_vel);
}


double JoyConverter::get_axis(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    int index) const
{
    if (!msg) {
        return 0.0;
    }

    if (index < 0) {
        return 0.0;
    }

    if (static_cast<size_t>(index) >= msg->axes.size()) {
        RCLCPP_WARN(
            this->get_logger(),
            "Axis index %d is out of range. axes size = %zu",
            index,
            msg->axes.size()
        );

        return 0.0;
    }

    return msg->axes[index];
}


double JoyConverter::apply_deadzone(double value) const
{
    if (std::abs(value) < this->deadzone_) {
        return 0.0;
    }

    return value;
}


void JoyConverter::publish_zero_velocity()
{
    if (!this->cmd_vel_publisher_) {
        return;
    }

    if (!this->cmd_vel_publisher_->is_activated()) {
        return;
    }


    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;


    this->cmd_vel_publisher_->publish(cmd_vel);
}


rcl_interfaces::msg::SetParametersResult
JoyConverter::parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    result.successful = true;
    result.reason = "success";


    for (const auto & param : parameters)
    {
        const auto & name = param.get_name();


        if (name == "input_topic" &&
            param.get_type() ==
            rclcpp::ParameterType::PARAMETER_STRING)
        {
            this->input_topic_ =
                param.as_string();
        }


        else if (name == "output_topic" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_STRING)
        {
            this->output_topic_ =
                param.as_string();
        }


        else if (name == "max_vx" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->max_vx_ =
                param.as_double();
        }


        else if (name == "max_vy" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->max_vy_ =
                param.as_double();
        }


        else if (name == "max_omega" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->max_omega_ =
                param.as_double();
        }


        else if (name == "axis_linear_x" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            this->axis_linear_x_ =
                param.as_int();
        }


        else if (name == "axis_linear_y" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            this->axis_linear_y_ =
                param.as_int();
        }


        else if (name == "axis_angular_z" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            this->axis_angular_z_ =
                param.as_int();
        }


        else if (name == "invert_linear_x" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_BOOL)
        {
            this->invert_linear_x_ =
                param.as_bool();
        }


        else if (name == "invert_linear_y" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_BOOL)
        {
            this->invert_linear_y_ =
                param.as_bool();
        }


        else if (name == "invert_angular_z" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_BOOL)
        {
            this->invert_angular_z_ =
                param.as_bool();
        }


        else if (name == "deadzone" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->deadzone_ =
                param.as_double();
        }


        else if (name == "joy_timeout" &&
                 param.get_type() ==
                 rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->joy_timeout_ =
                param.as_double();
        }
    }


    return result;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<JoyConverter>();

    rclcpp::spin(
        node->get_node_base_interface()
    );

    rclcpp::shutdown();

    return 0;
}