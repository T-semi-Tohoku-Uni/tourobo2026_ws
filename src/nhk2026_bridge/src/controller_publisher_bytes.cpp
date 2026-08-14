#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

using namespace std::chrono_literals;

class BytePublisher : public rclcpp::Node {
public:
  BytePublisher() : Node("controller_byte_publisher") {
    pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("motor0_cmd", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&BytePublisher::on_timer, this));
  }
private:
  void on_timer() {
    // 例: speed_target=1000.0 (LE float 00 00 7A 44), pid_mode=1, ctrl_mode=0
    std::vector<uint8_t> data = {0,0,122,68,1,0,0,0};
    auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();
    msg->data = data;
    pub_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "published %zu bytes", data.size());
  }
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BytePublisher>());
  rclcpp::shutdown();
  return 0;
}