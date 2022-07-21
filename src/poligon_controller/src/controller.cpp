// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "../dependencies/LowLevelController/include/LowLevelController.h"
#include "interfaces/msg/lamp_msg.hpp"
#include "interfaces/msg/angle_manipulator_msg.hpp"
#include "interfaces/msg/palletizer_msg.hpp"
#include "rcl_yaml_param_parser/parser.h"
#include "include/rapidjson/document.h"
#include "include/rapidjson/writer.h"
#include "include/rapidjson/stringbuffer.h"

using std::placeholders::_1;
class Address
{
private:
    string Ip;
    int Port;
public:
    string GetIp(){return Ip;}
    int GetPort(){return Port;}
  Address(const string ip, const int port) : Ip(ip), Port(port) {}
  Address(const string json_config, const string name)
  {
      auto address = GetAddress(json_config, name);
      Ip = address.Ip;
      Port = address.Port;
  }

  static Address GetAddress(const string json_config, const string name)
  {
      rapidjson::Document doc;
      doc.Parse(json_config.c_str());
      rapidjson::Value& val = doc[name.c_str()];
      return fromJSON(val);
  }
private:
  static Address fromJSON(const rapidjson::Value& doc) {
        if(!doc.IsObject())
            throw std::runtime_error("document should be an object");

        static const char* members[] = { "ip", "port" };
        for(size_t i = 0; i < sizeof(members)/sizeof(members[0]); i++)
            if(!doc.HasMember(members[i]))
                throw std::runtime_error("missing fields");

        string ip = doc["ip"].GetString();
        int port = doc["port"].GetInt();
        Address result(ip, port);
        return result;
    }
};

class Controller_node : public rclcpp::Node
{
public:
  Controller_node() : Node("controller")
  {
    
    Lamp1 = std::make_shared<LampController>(Lamp1Address.GetIp(), Lamp1Address.GetPort());
    if (!Lamp1->init())
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
    Lamp2 = std::make_shared<LampController>(Lamp2Address.GetIp(), Lamp2Address.GetPort());
    if (!Lamp2->init()) {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
    Lamp3 = std::make_shared<LampController>(Lamp3Address.GetIp(), Lamp3Address.GetPort());
    if (!Lamp3->init())
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
    Lamp4 = std::make_shared<LampController>(Lamp4Address.GetIp(), Lamp4Address.GetPort());
    if (!Lamp4->init())
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
    AngleManipulator = std::make_shared<AngleManipulatorController>(AngleManipulatorAddress.GetIp(), AngleManipulatorAddress.GetPort());
    if (!AngleManipulator->init())
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
    Palletizer = std::make_shared<PalletizerController>(PalletizerAddress.GetIp(), PalletizerAddress.GetPort());
    if (!Palletizer->init())
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }

    Lamp1_subscription = this->create_subscription<interfaces::msg::LampMsg>(
      "Lamp1", 10, std::bind(&Controller_node::lamp1_callback, this, _1));
    Lamp2_subscription = this->create_subscription<interfaces::msg::LampMsg>(
      "Lamp2", 10, std::bind(&Controller_node::lamp2_callback, this, _1));
    Lamp3_subscription = this->create_subscription<interfaces::msg::LampMsg>(
      "Lamp3", 10, std::bind(&Controller_node::lamp3_callback, this, _1));
    Lamp4_subscription = this->create_subscription<interfaces::msg::LampMsg>(
      "Lamp4", 10, std::bind(&Controller_node::lamp4_callback, this, _1));

    AngleManipulator_subscription = this->create_subscription<interfaces::msg::AngleManipulatorMsg>(
      "AngleManipulator", 10, std::bind(&Controller_node::AngleManipulator_callback, this, _1));
    Palletizer_subscription = this->create_subscription<interfaces::msg::PalletizerMsg>(
      "Palletizer", 10, std::bind(&Controller_node::Palletizer_callback, this, _1));
  }

private:
  void lamp1_callback(const interfaces::msg::LampMsg::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Lamp1:\nRed:\t'%d'\nOrange:\t'%d'\nBlue\t'%d'\nGreen\t'%d'", 
    message->red, message->orange, message->blue, message->green);
    if (!Lamp1->set(message->red, message->blue, message->green, message->orange))
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
  }
  void lamp2_callback(const interfaces::msg::LampMsg::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Lamp2:\nRed:\t'%d'\nOrange:\t'%d'\nBlue\t'%d'\nGreen\t'%d'", 
    message->red, message->orange, message->blue, message->green);
    if (!Lamp2->set(message->red, message->blue, message->green, message->orange))
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
  }
  void lamp3_callback(const interfaces::msg::LampMsg::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Lamp3:\nRed:\t'%d'\nOrange:\t'%d'\nBlue\t'%d'\nGreen\t'%d'", 
    message->red, message->orange, message->blue, message->green);
    if (!Lamp3->set(message->red, message->blue, message->green, message->orange))
    {
      perror("Sometthing went wrong\n");
      perror(strerror(errno));
    }
  }
  void lamp4_callback(const interfaces::msg::LampMsg::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Lamp4:\nRed:\t'%d'\nOrange:\t'%d'\nBlue\t'%d'\nGreen\t'%d'", 
    message->red, message->orange, message->blue, message->green);
    if (!Lamp4->set(message->red, message->blue, message->green, message->orange))
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
  rclcpp::Subscription<interfaces::msg::LampMsg>::SharedPtr Lamp4_subscription;
  rclcpp::Subscription<interfaces::msg::AngleManipulatorMsg>::SharedPtr AngleManipulator_subscription;
  rclcpp::Subscription<interfaces::msg::PalletizerMsg>::SharedPtr Palletizer_subscription;

  std::shared_ptr<LampController> Lamp1;
  std::shared_ptr<LampController> Lamp2;
  std::shared_ptr<LampController> Lamp3;
  std::shared_ptr<LampController> Lamp4;
  std::shared_ptr<AngleManipulatorController> AngleManipulator;
  std::shared_ptr<PalletizerController> Palletizer;

  Address Lamp1Address;
  Address Lamp2Address;
  Address Lamp3Address;
  Address Lamp4Address;
  Address AngleManipulatorAddress;
  Address PalletizerAddress;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller_node>());
  rclcpp::shutdown();
  return 0;
}
