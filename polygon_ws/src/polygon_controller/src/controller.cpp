
//Подключение библиотеки для работы с памятью
#include <memory>

// Подключение библиотеки rclcpp.hpp для получения доступа к ресурсам ros2
#include "rclcpp/rclcpp.hpp"

// Подключение библиотеки стандартного сообщения ros2 string
#include "std_msgs/msg/string.hpp"

// Подкючение библиотеки для управления устройствами
#include "../dependencies/LowLevelController/include/LowLevelController.h"

// Подключение нестандартных сообщений определенных в пакете interfaces
#include "interfaces/msg/lamp_msg.hpp"
#include "interfaces/msg/angle_manipulator_msg.hpp"
#include "interfaces/msg/palletizer_msg.hpp"

// Подключение библиотеки для работы с json данными
#include "../include/rapidjson/document.h"

// Подключение библиотеки для работы с файлами
#include "fstream"

// Подключение стандартной библиотеки unistd.h
#include "unistd.h"

using std::placeholders::_1;

// Функция которая считывает данные из файла в строку (std::string) и возващает эту строку
// Параметром file_path передается полный путь к файлу
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
  fin.close();
  return text;
}
// Структура которая хранит адрес(ip и port)
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

// Функция которая принимает строку (std::string) которая содержит json данные и считывает из него
// структуру Address с именем, которое передается параметром name 
// Передаваемая json строка должна иметь следующую структуру: 
// {"name1":{"ip":"XXX.XXX.XXX.XXX", "pot":XXXX},"name2":{"ip":"XXX.XXX.XXX.XXX", "pot":XXXX},...}
// параметр "name" и параметр "ip" имеют формат string, а параметр "port" имеет формат int
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

// Класс который реализует ноду (rclcpp::Node) "controller"
class Controller_node : public rclcpp::Node
{
public:
  // Конструктор класса
  Controller_node() : Node("controller")
  {
    // В строку config_path записывается полный путь в директорию, откуда запускается эта "нода"
    // Нода запускается из рабочей директории (из "workspace" - а) проекта, поэтому здесь будет путь
    // в рабочую директорию проекта
    std::string config_path(get_current_dir_name());

    // в config_path дописывается путь в файл конфигурации (config.json)
    config_path += "/src/polygon_controller/src/config.json";

    // В config_string записывается строка - считанная из файла конфигурации
    std::string config_string = ReadFile(config_path);
    RCLCPP_INFO(this->get_logger(), "Readed config:\n%s", config_string.c_str());
    
    // Инициализация полей которые хранят адреса устройств
    Lamp1Address = std::make_shared<Address>(GetAddress(config_string, "Lamp1"));
    Lamp2Address = std::make_shared<Address>(GetAddress(config_string, "Lamp2"));
    Lamp3Address = std::make_shared<Address>(GetAddress(config_string, "Lamp3"));
    AngleManipulatorAddress = std::make_shared<Address>(GetAddress(config_string, "AngleManipulator"));
    PalletizerAddress = std::make_shared<Address>(GetAddress(config_string, "Palletizer"));

    // Вывод на консоль информаций об инициализации адресов
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

    // инициализация полей которые хранят объекты классов управляемых устройств
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

    // Установка рабочей зоны манипуляторов
    Palletizer->setZone(
        PalletizerController::Position(0, -300, 160),
        PalletizerController::Position(300, 300, 290));
    AngleManipulator->setZone(
        AngleManipulatorController::Position(0, -300, 0, 0),
        AngleManipulatorController::Position(300, 300, 150, 90));

    // Подписка управляемых устройств в соответсятвующие им "топик" - и
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
  // Ниже определены функции "callback" - для управляемых устройств
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

  // Поля "Subscriber" - ы управляемых устройств
  // Сообщения, которые принимают эти "Subscriber" - ы определены в пакете interfaces
  rclcpp::Subscription<interfaces::msg::LampMsg>::SharedPtr Lamp1_subscription;
  rclcpp::Subscription<interfaces::msg::LampMsg>::SharedPtr Lamp2_subscription;
  rclcpp::Subscription<interfaces::msg::LampMsg>::SharedPtr Lamp3_subscription;
  rclcpp::Subscription<interfaces::msg::AngleManipulatorMsg>::SharedPtr AngleManipulator_subscription;
  rclcpp::Subscription<interfaces::msg::PalletizerMsg>::SharedPtr Palletizer_subscription;

  // Поля - "Умные" указатели на классы которые реализуют интерфейс управления соответствующими
  // устройствами
  // Эти классы определены и реализованиы в библиотеке LowLevelController.h
  std::shared_ptr<LampController> Lamp1;
  std::shared_ptr<LampController> Lamp2;
  std::shared_ptr<LampController> Lamp3;
  std::shared_ptr<AngleManipulatorController> AngleManipulator;
  std::shared_ptr<PalletizerController> Palletizer;

  // Поля для хранения адресов управляемых устройств
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
