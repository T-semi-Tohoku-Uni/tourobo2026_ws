#include "joy_changer.hpp"

#include <algorithm>
#include <functional>

using std::placeholders::_1;

namespace
{
constexpr size_t kCrossButton = 0;
constexpr size_t kCircleButton = 1;
constexpr size_t kTriangleButton = 2;
constexpr size_t kL1Button = 4;
constexpr size_t kR1Button = 5;
constexpr size_t kCreateButton = 8;
constexpr size_t kOptionButton = 9;
constexpr size_t kDpadVerticalAxis = 7;

bool button_pressed(const sensor_msgs::msg::Joy & joy, size_t index)
{
    return index < joy.buttons.size() && joy.buttons[index] != 0;
}

int32_t axis_direction(const sensor_msgs::msg::Joy & joy, size_t index)
{
    if (index >= joy.axes.size()) {
        return 0;
    }
    return joy.axes[index] > 0.0F ? 1 : (joy.axes[index] < 0.0F ? -1 : 0);
}
}  // namespace

JoyChanger::JoyChanger()
: LifecycleNode("joy_changer"),
  maxhand_rotation_(20),
  minhand_rotation_(-70),
  hand_rotation_speed_(0.0),
  hand_rotation_(0.0),
  has_last_joy_stamp_(false),
  maxball_holder_(0),
  max_ejectionrpm_(0)
{
    this->declare_parameter<int>("maxhand_rotation", 20);
    this->declare_parameter<int>("minhand_rotation", -70);
    this->declare_parameter<double>("hand_rotation_speed", 0.0);
    this->declare_parameter<int>("maxball_holder", 0);
    this->declare_parameter<int>("max_ejectionrpm", 0);

    maxhand_rotation_ = this->get_parameter("maxhand_rotation").as_int();
    minhand_rotation_ = this->get_parameter("minhand_rotation").as_int();
    hand_rotation_speed_ = this->get_parameter("hand_rotation_speed").as_double();
    maxball_holder_ = this->get_parameter("maxball_holder").as_int();
    max_ejectionrpm_ = this->get_parameter("max_ejectionrpm").as_int();

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&JoyChanger::parameters_callback, this, _1));
}

JoyChanger::CallbackReturn JoyChanger::on_configure(const rclcpp_lifecycle::State & state)
{
    air_cylinder_publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(
        "air_cylinder", rclcpp::SystemDefaultsQoS());
    robomasu_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "robomasu", rclcpp::SystemDefaultsQoS());
    ejection_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "ejection", rclcpp::SystemDefaultsQoS());
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", rclcpp::SystemDefaultsQoS(), std::bind(&JoyChanger::joy_callback, this, _1));

    RCLCPP_INFO(get_logger(), "on_configure() called. state: id=%u, label=%s",
        state.id(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

JoyChanger::CallbackReturn JoyChanger::on_activate(const rclcpp_lifecycle::State & state)
{
    has_last_joy_stamp_ = false;
    air_cylinder_publisher_->on_activate();
    robomasu_publisher_->on_activate();
    ejection_publisher_->on_activate();
    RCLCPP_INFO(get_logger(), "on_activate() called. state: id=%u, label=%s",
        state.id(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

JoyChanger::CallbackReturn JoyChanger::on_deactivate(const rclcpp_lifecycle::State & state)
{
    has_last_joy_stamp_ = false;
    air_cylinder_publisher_->on_deactivate();
    robomasu_publisher_->on_deactivate();
    ejection_publisher_->on_deactivate();
    RCLCPP_INFO(get_logger(), "on_deactivate() called. state: id=%u, label=%s",
        state.id(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

JoyChanger::CallbackReturn JoyChanger::on_cleanup(const rclcpp_lifecycle::State & state)
{
    joy_subscriber_.reset();
    air_cylinder_publisher_.reset();
    robomasu_publisher_.reset();
    ejection_publisher_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup() called. state: id=%u, label=%s",
        state.id(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

JoyChanger::CallbackReturn JoyChanger::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_ERROR(get_logger(), "on_error() called. state: id=%u, label=%s",
        state.id(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

JoyChanger::CallbackReturn JoyChanger::on_shutdown(const rclcpp_lifecycle::State & state)
{
    joy_subscriber_.reset();
    air_cylinder_publisher_.reset();
    robomasu_publisher_.reset();
    ejection_publisher_.reset();
    RCLCPP_INFO(get_logger(), "on_shutdown() called. state: id=%u, label=%s",
        state.id(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

void JoyChanger::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    if (!air_cylinder_publisher_->is_activated()) {
        return;
    }

    // bit 0: circle, bit 1: cross, bit 2: create, bit 3: option (share-equivalent input)
    uint8_t air_cylinder = 0;
    air_cylinder |= button_pressed(*joy, kCircleButton) << 0;
    air_cylinder |= button_pressed(*joy, kCrossButton) << 1;
    air_cylinder |= button_pressed(*joy, kCreateButton) << 2;
    air_cylinder |= button_pressed(*joy, kOptionButton) << 3;
    std_msgs::msg::ByteMultiArray air_cylinder_msg;
    air_cylinder_msg.data = {air_cylinder};

    const bool l1 = button_pressed(*joy, kL1Button);
    const bool r1 = button_pressed(*joy, kR1Button);
    const int32_t hand_direction = l1 == r1 ? 0 : (l1 ? 1 : -1);
    const rclcpp::Time joy_stamp(joy->header.stamp);
    if (has_last_joy_stamp_) {
        const double elapsed_seconds = (joy_stamp - last_joy_stamp_).seconds();
        if (elapsed_seconds > 0.0) {
            hand_rotation_ += hand_direction * hand_rotation_speed_ * elapsed_seconds;
        }
    }
    last_joy_stamp_ = joy_stamp;
    has_last_joy_stamp_ = true;
    const double hand_rotation_min = std::min(
        static_cast<double>(minhand_rotation_), static_cast<double>(maxhand_rotation_));
    const double hand_rotation_max = std::max(
        static_cast<double>(minhand_rotation_), static_cast<double>(maxhand_rotation_));
    hand_rotation_ = std::clamp(hand_rotation_, hand_rotation_min, hand_rotation_max);

    std_msgs::msg::Int32MultiArray robomasu_msg;
    robomasu_msg.data = {
        static_cast<int32_t>(hand_rotation_),
        axis_direction(*joy, kDpadVerticalAxis) * maxball_holder_};

    std_msgs::msg::Int32MultiArray ejection_msg;
    ejection_msg.data = {button_pressed(*joy, kTriangleButton) ? max_ejectionrpm_ : 0};

    air_cylinder_publisher_->publish(air_cylinder_msg);
    robomasu_publisher_->publish(robomasu_msg);
    ejection_publisher_->publish(ejection_msg);
}

rcl_interfaces::msg::SetParametersResult JoyChanger::parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto & parameter : parameters) {
        if (parameter.get_name() == "maxhand_rotation") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                result.successful = false;
                result.reason = "maxhand_rotation must be an integer";
                return result;
            }
            maxhand_rotation_ = parameter.as_int();
        } else if (parameter.get_name() == "minhand_rotation") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                result.successful = false;
                result.reason = "minhand_rotation must be an integer";
                return result;
            }
            minhand_rotation_ = parameter.as_int();
        } else if (parameter.get_name() == "hand_rotation_speed") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || parameter.as_double() < 0.0) {
                result.successful = false;
                result.reason = "hand_rotation_speed must be a non-negative double";
                return result;
            }
            hand_rotation_speed_ = parameter.as_double();
        } else if (parameter.get_name() == "maxball_holder") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                continue;
            }
            maxball_holder_ = parameter.as_int();
        } else if (parameter.get_name() == "max_ejectionrpm") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                continue;
            }
            max_ejectionrpm_ = parameter.as_int();
        }
    }
    return result;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyChanger>()->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
