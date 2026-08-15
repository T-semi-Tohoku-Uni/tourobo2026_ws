#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <vector>
using namespace std::chrono_literals;

class BytePublisher : public rclcpp::Node {
public:
  BytePublisher() : Node("controller_byte_publisher") {
    pub0_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("motor0_cmd", 10);
    pub1_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("motor1_cmd", 10);
    pub2_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("motor2_cmd", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&BytePublisher::on_timer, this));
  }
private:
  void on_timer() {
    // 例: vx = 100 mm/s -> 0x0064 -> [0x64,0x00], 他は0, pid_mode=1, ctrl=0
    std::vector<uint8_t> data = {0x64,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00};
    auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();
    msg->data = data;
    pub0_->publish(*msg);
    pub1_->publish(*msg);
    pub2_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "published %zu bytes to 3 topics", data.size());
  }
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub0_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub2_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BytePublisher>());
  rclcpp::shutdown();
  return 0;
}