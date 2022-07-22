#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "../dependencies/LowLevelController/include/LowLevelController.h"
#include "interfaces/msg/lamp_msg.hpp"
#include "interfaces/msg/angle_manipulator_msg.hpp"
#include "interfaces/msg/palletizer_msg.hpp"
#include "../include/rapidjson/document.h"
#include "fstream"
#include "unistd.h"

using std::placeholders::_1;

std::string ReadFile(const std::string file_path)
{
  ifstream fin;
  fin.open(file_path.c_str());
  std::string text;

  if(!fin.is_open())
  {
    perror("Sometthing went wrong\n");
    perror(strerror(errno));
  }
  std::string str;
  while(!fin.eof())
  {
    getline(fin, str);
    text += str;
    text += "\n";
  }
  return text;
}
struct Address
{
private:
  std::string Ip;
  int Port;
public:
  std::string GetIp(){return Ip;}
  int GetPort(){return Port;}
  Address(const string ip, const int port) : Ip(ip), Port(port) {}
};

Address GetAddress(const string json_config_string, const string name)
{
  rapidjson::Document doc;
  doc.Parse(json_config_string.c_str());
  rapidjson::Value& json_value = doc[name.c_str()];
  if(!json_value.IsObject())
      throw std::runtime_error("document should be an object");

    static const char* members[] = { "ip", "port" };
    for(size_t i = 0; i < sizeof(members)/sizeof(members[0]); i++)
      if(!json_value.HasMember(members[i]))
        throw std::runtime_error("missing fields");
        
    std::string ip(json_value["ip"].GetString());
    int port = json_value["port"].GetInt();
    Address result(ip, port);
    return result;
}
class Controller_node : public rclcpp::Node
{
public:
  Controller_node() : Node("controller")
  {
    std::string config_path(get_current_dir_name());
    config_path += "/src/polygon_controller/src/config.json";
    std::string config_string = ReadFile(config_path);
    RCLCPP_INFO(this->get_logger(), "Readed config:\n%s", config_string.c_str());
    
    Lamp1Address = std::make_shared<Address>(GetAddress(config_string, "Lamp1"));
    Lamp2Address = std::make_shared<Address>(GetAddress(config_string, "Lamp2"));
    Lamp3Address = std::make_shared<Address>(GetAddress(config_string, "Lamp3"));
    AngleManipulatorAddress = std::make_shared<Address>(GetAddress(config_string, "AngleManipulator"));
    PalletizerAddress = std::make_shared<Address>(GetAddress(config_string, "Palletizer"));

    RCLCPP_INFO(this->get_logger(), "\n\nLamp1 address changed:\nip: '%s'\tport: '%d'\n",
      Lamp1Address->GetIp().c_str(), Lamp1Address->GetPort());
    RCLCPP_INFO(this->get_logger(), "\n\nLamp2 address changed:\nip: '%s'\tport: '%d'\n",
      Lamp2Address->GetIp().c_str(), Lamp2Address->GetPort());
    RCLCPP_INFO(this->get_logger(), "\n\nLamp3 address changed:\nip: '%s'\tport: '%d'\n",
      Lamp3Address->GetIp().c_str(), Lamp3Address->GetPort());
    RCLCPP_INFO(this->get_logger(), "\n\nAngleManipulator address changed:\nip: '%s'\tport: '%d'\n",
      AngleManipulatorAddress->GetIp().c_str(), AngleManipulatorAddress->GetPort());
    RCLCPP_INFO(this->get_logger(), "\n\nPalletizer address changed:\nip: '%s'\tport: '%d'\n",
      PalletizerAddress->GetIp().c_str(), PalletizerAddress->GetPort());
    Lamp1 = std::make_shared<LampController>(Lamp1Address->GetIp(), Lamp1Address->GetPort());
    Lamp1->init();
    Lamp2 = std::make_shared<LampController>(Lamp2Address->GetIp(), Lamp2Address->GetPort());
    Lamp2->init();
    Lamp3 = std::make_shared<LampController>(Lamp3Address->GetIp(), Lamp3Address->GetPort());
    Lamp3->init();
    AngleManipulator = std::make_shared<AngleManipulatorController>(AngleManipulatorAddress->GetIp(), AngleManipulatorAddress->GetPort());
    AngleManipulator->init();
    Palletizer = std::make_shared<PalletizerController>(PalletizerAddress->GetIp(), PalletizerAddress->GetPort());
    Palletizer->init();

    Palletizer->setZone(
        PalletizerController::Position(0, -300, 160),
        PalletizerController::Position(300, 300, 290));
    AngleManipulator->setZone(
        AngleManipulatorController::Position(0, -300, 0, 0),
        AngleManipulatorController::Position(300, 300, 150, 90));

    Lamp1_subscription = this->create_subscription<interfaces::msg::LampMsg>(
      "Lamp1", 10, std::bind(&Controller_node::lamp1_callback, this, _1));
    Lamp2_subscription = this->create_subscription<interfaces::msg::LampMsg>(
      "Lamp2", 10, std::bind(&Controller_node::lamp2_callback, this, _1));
    Lamp3_subscription = this->create_subscription<interfaces::msg::LampMsg>(
      "Lamp3", 10, std::bind(&Controller_node::lamp3_callback, this, _1));

    AngleManipulator_subscription = this->create_subscription<interfaces::msg::AngleManipulatorMsg>(
      "AngleManipulator", 10, std::bind(&Controller_node::AngleManipulator_callback, this, _1));
    Palletizer_subscription = this->create_subscription<interfaces::msg::PalletizerMsg>(
      "Palletizer", 10, std::bind(&Controller_node::Palletizer_callback, this, _1));
  }

private:
  void lamp1_callback(const interfaces::msg::LampMsg::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Lamp1:\nRed:\t'%d'\nOrange:\t'%d'\nBlue\t'%d'\nGreen\t'%d'", 
    msg->red, msg->orange, msg->blue, msg->green);
    if (!Lamp1->set(msg->red, msg->blue, msg->green, msg->orange))
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
  }

  void lamp2_callback(const interfaces::msg::LampMsg::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Lamp2:\nRed:\t'%d'\nOrange:\t'%d'\nBlue\t'%d'\nGreen\t'%d'", 
    msg->red, msg->orange, msg->blue, msg->green);
    if (!Lamp2->set(msg->red, msg->blue, msg->green, msg->orange))
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
  }

  void lamp3_callback(const interfaces::msg::LampMsg::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Lamp3:\nRed:\t'%d'\nOrange:\t'%d'\nBlue\t'%d'\nGreen\t'%d'", 
    msg->red, msg->orange, msg->blue, msg->green);
    if (!Lamp3->set(msg->red, msg->blue, msg->green, msg->orange))
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
  }

  void AngleManipulator_callback(const interfaces::msg::AngleManipulatorMsg::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Angle Manipulator goal position:\nX:\t'%f\nY:\t'%f\nZ:\t'%f\nangle:\t'%f'\npomp state:\t'%d'",
    msg->x, msg->y, msg->z, msg->angle, msg->pomp_state);
    if (!AngleManipulator->moveToAndChangePompState(msg->x, msg->y, msg->z, msg->angle, msg->pomp_state))
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
  }

  void Palletizer_callback(const interfaces::msg::PalletizerMsg::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Palletizer goal position:\nX:\t'%f\nY:\t'%f\nZ:\t'%f\npomp state:\t'%d'",
    msg->x, msg->y, msg->z, msg->pomp_state);
    if (!Palletizer->moveToAndChangePompState(msg->x, msg->y, msg->z, msg->pomp_state))
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
  }

  rclcpp::Subscription<interfaces::msg::LampMsg>::SharedPtr Lamp1_subscription;
  rclcpp::Subscription<interfaces::msg::LampMsg>::SharedPtr Lamp2_subscription;
  rclcpp::Subscription<interfaces::msg::LampMsg>::SharedPtr Lamp3_subscription;
  rclcpp::Subscription<interfaces::msg::AngleManipulatorMsg>::SharedPtr AngleManipulator_subscription;
  rclcpp::Subscription<interfaces::msg::PalletizerMsg>::SharedPtr Palletizer_subscription;

  std::shared_ptr<LampController> Lamp1;
  std::shared_ptr<LampController> Lamp2;
  std::shared_ptr<LampController> Lamp3;
  std::shared_ptr<AngleManipulatorController> AngleManipulator;
  std::shared_ptr<PalletizerController> Palletizer;

  std::shared_ptr<Address> Lamp1Address;
  std::shared_ptr<Address> Lamp2Address;
  std::shared_ptr<Address> Lamp3Address;
  std::shared_ptr<Address> Lamp4Address;
  std::shared_ptr<Address> AngleManipulatorAddress;
  std::shared_ptr<Address> PalletizerAddress;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller_node>());
  rclcpp::shutdown();
  return 0;
}
