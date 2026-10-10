#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "my_msg_package/msg/my_msg.hpp"

using namespace std::chrono_literals;

class DataGenerator : public rclcpp::Node
{
public:
  DataGenerator()
  : Node("data_generator"), count_(0)
  {
    publisher_ = this->create_publisher<my_msg_package::msg::MyMsg>("counter", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&DataGenerator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = my_msg_package::msg::MyMsg();
    message.counter = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: %d, %s",
      message.counter, message.comment.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_msg_package::msg::MyMsg>::SharedPtr publisher_;
  int32_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataGenerator>());
  rclcpp::shutdown();
  return 0;
}
