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

class Publisher_node : public rclcpp::Node
{
public:
  Publisher_node()
  : Node("publisher_node"), count_(0)
  {
    Lamp1Publisher = this->create_publisher<interfaces::msg::LampMsg>("Lamp1", 10);
    Lamp2Publisher = this->create_publisher<interfaces::msg::LampMsg>("Lamp2", 10);
    Lamp3Publisher = this->create_publisher<interfaces::msg::LampMsg>("Lamp3", 10);
    AngleManipulatorPublisher = this->create_publisher<interfaces::msg::AngleManipulatorMsg>("AngleManipulator", 10);
    PalletizerPublisher = this->create_publisher<interfaces::msg::PalletizerMsg>("Palletizer", 10);
    timer_ = this->create_wall_timer(
      300ms, std::bind(&Publisher_node::timer_callback, this));
    demo_timer = this->create_wall_timer(
      3000ms, std::bind(&Publisher_node::demo_timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = interfaces::msg::LampMsg();

    if(count_ % 4 == 0)
      msg.red = true;
    if(count_ % 4 == 1)
      msg.orange = true;
    if(count_ % 4 == 2)
      msg.blue = count_;
    if(count_ % 4 == 3)
      msg.green = count_;
    count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing:\nRed:\t'%d'\nOrange:\t'%d'\nBlue\t'%d'\nGreen\t'%d'", 
    msg.red, msg.orange, msg.blue, msg.green);

    Lamp1Publisher->publish(msg);
    Lamp2Publisher->publish(msg);
    Lamp3Publisher->publish(msg);
  }

  void demo_timer_callback()
  {
    interfaces::msg::AngleManipulatorMsg amsg;
    interfaces::msg::PalletizerMsg pmsg;
    if (kek % 2 == 0)
    {
      amsg.x = 150;
      amsg.y = 150;
      amsg.z = 140;
      amsg.angle = 45;
    
      pmsg.x = 200;
      pmsg.y = 0;
      pmsg.z = 250;
    }
    else
    {
      amsg.x = 250;
      amsg.y = -50;
      amsg.z = 40;
      amsg.angle = 45;
    
      pmsg.x = 200;
      pmsg.y = -150;
      pmsg.z = 170;
    }
    kek++;
    AngleManipulatorPublisher->publish(amsg);
    PalletizerPublisher->publish(pmsg);

  }
  int X = 0;
  int Y = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr demo_timer;
  rclcpp::Publisher<interfaces::msg::LampMsg>::SharedPtr Lamp1Publisher;
  rclcpp::Publisher<interfaces::msg::LampMsg>::SharedPtr Lamp2Publisher;
  rclcpp::Publisher<interfaces::msg::LampMsg>::SharedPtr Lamp3Publisher;
  rclcpp::Publisher<interfaces::msg::AngleManipulatorMsg>::SharedPtr AngleManipulatorPublisher;
  rclcpp::Publisher<interfaces::msg::PalletizerMsg>::SharedPtr PalletizerPublisher;
  int count_ = 4;
  int kek = 2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher_node>());
  rclcpp::shutdown();
  return 0;
}
