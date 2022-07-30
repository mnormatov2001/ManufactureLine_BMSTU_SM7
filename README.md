## ManufactureLine_BMSTU_SM7
### Проект управления производственной линией учебного стенда кафедры СМ7 МГТУ им. Н.Э. Баумана
Для запуска проекта на компьютере должен быть установлен Ros2 и система сборки Colcon.  
  
  Проект состоит из трех пакетов: interfaces, polygon_controller и publisher. В пакете interfaces определены типы сообщений
получаемых "Subscriber" - ами устройств. В пакете polygon_controller определена "нода" Controller.
В этой "ноде" создаются "Subscriber" - ы для каждого устройства и они подписываются на соответствующие им "топики".
В пакете publisher определена "нода" publisher_node. В этой "ноде" создаются "publisher" - ы для устройств и они привязываются
к соответствующим им "топикам". А также в этом топике создаются два таймера - "callback" функции которых публикуют сообщения "publisher" - ами устройств.  
  
  Порядок запуска:
  1. Скачиваем проект (если не скачан)
  ```bash
  git clone https://github.com/mnormatov2001/ManufactureLine_BMSTU_SM7.git
  ```
  2. Переходим в рабочую директорию:
  ```bash
  cd ManufactureLine_BMSTU_SM7/polygon_ws
  ```
  3. Собираем проект:
  ```bash
  colcon build
  ```
  4. Инициализируем рабочее пространство:
  ```bash
  . install/setup.bash
  ```
  5. Запускаем "ноду" controller:
  ```bash
  ros2 run polygon_controller controller
  ```
  6. Открываем новый терминал и переходим в рабочую директорию
  7. Инициализируем рабочее пространство:
  ```bash
  . install/setup.bash
  ```
  8. Запускаем "ноду" publisher_node:
  ```bash
  ros2 run publisher publisher_node
  ```
