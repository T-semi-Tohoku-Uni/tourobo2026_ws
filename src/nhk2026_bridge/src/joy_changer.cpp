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

constexpr size_t kAirCylinderCircle = 0;
constexpr size_t kAirCylinderCross = 1;
constexpr size_t kHolderServoCreate = 2;
constexpr size_t kHolderServoOption = 3;
constexpr size_t kEjectionTriangle = 4;

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
    ejectionRpm_max_(0),
    ejection_state_(0),
    air_cylinder_state_(0),
    holder_servo1_state_(0),
    holder_servo3_state_(0),
    button_was_pressed_{false, false, false, false, false},
    has_button_release_stamp_{false, false, false, false, false}
{
    this->declare_parameter<int>("handRotation_max", 20);
    this->declare_parameter<int>("handRotation_min", -70);
    this->declare_parameter<double>("handRotation_speed", 0.0);
    this->declare_parameter<int>("ballHolder_max", 0);
    this->declare_parameter<int>("ballHolder_min", 0);
    this->declare_parameter<double>("ballHolder_speed", 0.0);
    this->declare_parameter<int>("ejectionRpm_max", 0);
    this->declare_parameter<double>("ejectionRpm_rate", 1.0);
    this->declare_parameter<double>("toggleDebounceSeconds", 0.1);
    this->declare_parameter<int>("holder_servo1_0", 130);
    this->declare_parameter<int>("holder_servo1_1", 88);
    this->declare_parameter<int>("holder_servo1_2", 50);
    this->declare_parameter<int>("holder_servo3_0", 20);
    this->declare_parameter<int>("holder_servo3_1", 62);

    handRotation_max_ = this->get_parameter("handRotation_max").as_int();
    handRotation_min_ = this->get_parameter("handRotation_min").as_int();
    handRotation_speed_ = this->get_parameter("handRotation_speed").as_double();
    ballHolder_max_ = this->get_parameter("ballHolder_max").as_int();
    ballHolder_min_ = this->get_parameter("ballHolder_min").as_int();
    ballHolder_speed_ = this->get_parameter("ballHolder_speed").as_double();
    ejectionRpm_max_ = this->get_parameter("ejectionRpm_max").as_int();
    ejectionRpm_rate_ = this->get_parameter("ejectionRpm_rate").as_double();
    toggleDebounceSeconds_ = this->get_parameter("toggleDebounceSeconds").as_double();
    holder_servo1_0_ = this->get_parameter("holder_servo1_0").as_int();
    holder_servo1_1_ = this->get_parameter("holder_servo1_1").as_int();
    holder_servo1_2_ = this->get_parameter("holder_servo1_2").as_int();
    holder_servo3_0_ = this->get_parameter("holder_servo3_0").as_int();
    holder_servo3_1_ = this->get_parameter("holder_servo3_1").as_int();

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
    std_msgs::msg::ByteMultiArray air_cylinder_msg;
    air_cylinder_msg.data = {air_cylinder_state_};

        if (check_toggle_button_debounce(
            button_pressed(*joy, kCreateButton), kHolderServoCreate, joy_stamp)) {
        holder_servo1_state_ = (holder_servo1_state_ + 1) % 3; // rotate through 0, 1, 2
    }
    if (check_toggle_button_debounce(
            button_pressed(*joy, kOptionButton), kHolderServoOption, joy_stamp)) {
        holder_servo1_state_ = (holder_servo1_state_ + 1) % 2; // rotate through 0, 1
    }

    const int32_t holder_servo1_value = (holder_servo1_state_ == 0) ? holder_servo1_0_ :
                                    (holder_servo1_state_ == 1) ? holder_servo1_1_ : holder_servo1_2_;
    const int32_t holder_servo3_value = (holder_servo3_state_ == 0) ? holder_servo3_0_ : holder_servo3_1_;
    std_msgs::msg::Int32MultiArray holder_servo_msg;
    holder_servo_msg.data = {
        holder_servo1_value,
        holder_servo3_value
    };

    const bool l1 = button_pressed(*joy, kL1Button);
    const bool r1 = button_pressed(*joy, kR1Button);
    const int32_t hand_direction = l1 == r1 ? 0 : (l1 ? 1 : -1);
    const int32_t ballHolder_direction = (-1)*axis_direction(*joy, kDpadVerticalAxis);
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

    if (check_toggle_button_debounce(
            button_pressed(*joy, kTriangleButton), kEjectionTriangle, joy_stamp)) {
        ejection_state_ = (ejection_state_ + 1) % 2; // toggle between 0 and 1
    }
    

    std_msgs::msg::Int32MultiArray ejection_msg;

    if (ejection_state_ == 1) {
        const float ejectionRpm_max_f = static_cast<float>(ejectionRpm_max_);
        ejection_msg.data = {ejectionRpm_max_, static_cast<int32_t>(ejectionRpm_max_f * ejectionRpm_rate_), static_cast<int32_t>(ejectionRpm_max_f * ejectionRpm_rate_)};
    } else {
        ejection_msg.data = {0, 0, 0};
    }

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
            (stamp - last_button_release_stamp_[index]).seconds() >= toggleDebounceSeconds_;
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

// This function is used to check if the toggle button has been pressed and released. updata_toggle_button()と同じ処理を行うが、戻り値でトグルボタンが押されたかどうかを返す
bool JoyChanger::check_toggle_button_debounce(
bool pressed, size_t index, const rclcpp::Time & stamp)
{
    bool debounce_elapsed = false;
    if (pressed && !button_was_pressed_[index]) {
        debounce_elapsed = !has_button_release_stamp_[index] ||
            (stamp - last_button_release_stamp_[index]).seconds() >= toggleDebounceSeconds_;
    }

    if (!pressed && button_was_pressed_[index]) {
        last_button_release_stamp_[index] = stamp;
        has_button_release_stamp_[index] = true;
    }
    button_was_pressed_[index] = pressed;
    return debounce_elapsed;
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
        else if (parameter.get_name() == "ejectionRpm_max") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                continue;
            }
            ejectionRpm_max_ = parameter.as_int();
        }
        else if (parameter.get_name() == "ejectionRpm_rate") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || parameter.as_double() < 0.0) {
                result.successful = false;
                result.reason = "ejectionRpm_rate must be a non-negative double";
                return result;
            }
            ejectionRpm_rate_ = parameter.as_double();
        }
        else if (parameter.get_name() == "toggleDebounceSeconds") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || parameter.as_double() < 0.0) {
                continue;
            }
            toggleDebounceSeconds_ = parameter.as_double();
        }
        else if (parameter.get_name() == "holder_servo1_0") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                continue;
            }
            holder_servo1_0_ = parameter.as_int();
        }
        else if (parameter.get_name() == "holder_servo1_1") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                continue;
            }
            holder_servo1_1_ = parameter.as_int();
        }
        else if (parameter.get_name() == "holder_servo1_2") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                continue;
            }
            holder_servo1_2_ = parameter.as_int();
        }
        else if (parameter.get_name() == "holder_servo3_0") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                continue;
            }
            holder_servo3_0_ = parameter.as_int();
        }
        else if (parameter.get_name() == "holder_servo3_1") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                continue;
            }
            holder_servo3_1_ = parameter.as_int();
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
