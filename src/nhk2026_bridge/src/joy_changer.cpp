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
constexpr double kToggleDebounceSeconds = 0.1;

constexpr size_t kAirCylinderCircle = 0;
constexpr size_t kAirCylinderCross = 1;
constexpr size_t kHolderServoCreate = 2;
constexpr size_t kHolderServoOption = 3;

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
    handRotation_max_(20),
    handRotation_min_(-70),
    handRotation_speed_(0.0),
    hand_rotation_(0.0),
    has_last_joy_stamp_(false),
    ballHolder_max_(0),
    ballHolder_min_(0),
    ballHolder_speed_(0.0),
    ballHolder_(0.0),
    max_ejectionrpm_(0),
    air_cylinder_state_(0),
    holder_servo_state_(0),
    button_was_pressed_{false, false, false, false},
    has_button_release_stamp_{false, false, false, false}
{
    this->declare_parameter<int>("handRotation_max", 20);
    this->declare_parameter<int>("handRotation_min", -70);
    this->declare_parameter<double>("handRotation_speed", 0.0);
    this->declare_parameter<int>("ballHolder_max", 0);
    this->declare_parameter<int>("ballHolder_min", 0);
    this->declare_parameter<double>("ballHolder_speed", 0.0);
    this->declare_parameter<int>("max_ejectionrpm", 0);

    handRotation_max_ = this->get_parameter("handRotation_max").as_int();
    handRotation_min_ = this->get_parameter("handRotation_min").as_int();
    handRotation_speed_ = this->get_parameter("handRotation_speed").as_double();
    ballHolder_max_ = this->get_parameter("ballHolder_max").as_int();
    ballHolder_min_ = this->get_parameter("ballHolder_min").as_int();
    ballHolder_speed_ = this->get_parameter("ballHolder_speed").as_double();
    max_ejectionrpm_ = this->get_parameter("max_ejectionrpm").as_int();

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&JoyChanger::parameters_callback, this, _1));
}

JoyChanger::CallbackReturn JoyChanger::on_configure(const rclcpp_lifecycle::State & state)
{
    air_cylinder_publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(
        "air_cylinder", rclcpp::SystemDefaultsQoS());
    holder_servo_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "holder_servo", rclcpp::SystemDefaultsQoS());
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
    holder_servo_publisher_->on_activate();
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
    holder_servo_publisher_->on_deactivate();
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
    holder_servo_publisher_.reset();
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
    holder_servo_publisher_.reset();
    robomasu_publisher_.reset();
    ejection_publisher_.reset();
    RCLCPP_INFO(get_logger(), "on_shutdown() called. state: id=%u, label=%s",
        state.id(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

void JoyChanger::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    if (!air_cylinder_publisher_->is_activated() || !holder_servo_publisher_->is_activated()) {
        return;
    }

    const rclcpp::Time joy_stamp(joy->header.stamp);
    update_toggle_button(
        button_pressed(*joy, kCircleButton), kAirCylinderCircle, air_cylinder_state_, 0, joy_stamp);
    update_toggle_button(
        button_pressed(*joy, kCrossButton), kAirCylinderCross, air_cylinder_state_, 1, joy_stamp);
    update_toggle_button(
        button_pressed(*joy, kCreateButton), kHolderServoCreate, holder_servo_state_, 0, joy_stamp);
    update_toggle_button(
        button_pressed(*joy, kOptionButton), kHolderServoOption, holder_servo_state_, 1, joy_stamp);

    std_msgs::msg::ByteMultiArray air_cylinder_msg;
    air_cylinder_msg.data = {air_cylinder_state_};
    std_msgs::msg::Int32MultiArray holder_servo_msg;
    holder_servo_msg.data = {
        static_cast<int32_t>(holder_servo_state_ & 0x01),
        static_cast<int32_t>((holder_servo_state_ & 0x02) >> 1)
    };

    const bool l1 = button_pressed(*joy, kL1Button);
    const bool r1 = button_pressed(*joy, kR1Button);
    const int32_t hand_direction = l1 == r1 ? 0 : (l1 ? 1 : -1);
    const int32_t ballHolder_direction = axis_direction(*joy, kDpadVerticalAxis);
    if (has_last_joy_stamp_) {
        const double elapsed_seconds = (joy_stamp - last_joy_stamp_).seconds();
        if (elapsed_seconds > 0.0) {
            hand_rotation_ += hand_direction * handRotation_speed_ * elapsed_seconds;
            ballHolder_ += ballHolder_direction * ballHolder_speed_ * elapsed_seconds;
        }
    }
    last_joy_stamp_ = joy_stamp;
    has_last_joy_stamp_ = true;
    const double hand_rotation_min = std::min(
        static_cast<double>(handRotation_min_), static_cast<double>(handRotation_max_));
    const double hand_rotation_max = std::max(
        static_cast<double>(handRotation_min_), static_cast<double>(handRotation_max_));
    hand_rotation_ = std::clamp(hand_rotation_, hand_rotation_min, hand_rotation_max);

    const double ballHolder_min = std::min(
        static_cast<double>(ballHolder_min_), static_cast<double>(ballHolder_max_));
    const double ballHolder_max = std::max(
        static_cast<double>(ballHolder_min_), static_cast<double>(ballHolder_max_));
    ballHolder_ = std::clamp(ballHolder_, ballHolder_min, ballHolder_max);

    std_msgs::msg::Int32MultiArray robomasu_msg;
    robomasu_msg.data = {
        static_cast<int32_t>(hand_rotation_),
        static_cast<int32_t>(ballHolder_)
    };

    std_msgs::msg::Int32MultiArray ejection_msg;
    ejection_msg.data = {button_pressed(*joy, kTriangleButton) ? max_ejectionrpm_ : 0};

    air_cylinder_publisher_->publish(air_cylinder_msg);
    holder_servo_publisher_->publish(holder_servo_msg);
    robomasu_publisher_->publish(robomasu_msg);
    ejection_publisher_->publish(ejection_msg);
}

void JoyChanger::update_toggle_button(
    bool pressed, size_t index, uint8_t & output, uint8_t bit, const rclcpp::Time & stamp)
{
    if (pressed && !button_was_pressed_[index]) {
        const bool debounce_elapsed = !has_button_release_stamp_[index] ||
            (stamp - last_button_release_stamp_[index]).seconds() >= kToggleDebounceSeconds;
        if (debounce_elapsed) {
            output ^= static_cast<uint8_t>(1U << bit);
        }
    }

    if (!pressed && button_was_pressed_[index]) {
        last_button_release_stamp_[index] = stamp;
        has_button_release_stamp_[index] = true;
    }
    button_was_pressed_[index] = pressed;
}

rcl_interfaces::msg::SetParametersResult JoyChanger::parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto & parameter : parameters) {
        if (parameter.get_name() == "handRotation_max") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                result.successful = false;
                result.reason = "handRotation_max must be an integer";
                return result;
            }
            handRotation_max_ = parameter.as_int();
        } else if (parameter.get_name() == "handRotation_min") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                result.successful = false;
                result.reason = "handRotation_min must be an integer";
                return result;
            }
            handRotation_min_ = parameter.as_int();
        } else if (parameter.get_name() == "handRotation_speed") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || parameter.as_double() < 0.0) {
                result.successful = false;
                result.reason = "handRotation_speed must be a non-negative double";
                return result;
            }
            handRotation_speed_ = parameter.as_double();
        } else if (parameter.get_name() == "ballHolder_max") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                continue;
            }
            ballHolder_max_ = parameter.as_int();
        } else if (parameter.get_name() == "ballHolder_min") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                continue;
            }
            ballHolder_min_ = parameter.as_int();
        } else if (parameter.get_name() == "ballHolder_speed") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || parameter.as_double() < 0.0) {
                continue;
            }
            ballHolder_speed_ = parameter.as_double();
        }
        else if (parameter.get_name() == "max_ejectionrpm") {
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
