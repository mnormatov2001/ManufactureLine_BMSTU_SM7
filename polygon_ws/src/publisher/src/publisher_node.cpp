#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/lamp_msg.hpp"
#include "interfaces/msg/angle_manipulator_msg.hpp"
#include "interfaces/msg/palletizer_msg.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("publisher_node"), count_(0)
  {
    publisher_ = this->create_publisher<interfaces::msg::LampMsg>("Lamp1", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = interfaces::msg::LampMsg();
    msg.red = count_;
    msg.orange = count_;
    msg.blue = count_;
    msg.green = count_;
    count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing:\nRed:\t'%d'\nOrange:\t'%d'\nBlue\t'%d'\nGreen\t'%d'", 
    msg.red, msg.orange, msg.blue, msg.green);

    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::LampMsg>::SharedPtr publisher_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
