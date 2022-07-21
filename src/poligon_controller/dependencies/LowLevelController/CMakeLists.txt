cmake_minimum_required(VERSION 3.16)

include_directories(include)

set(HEADERS "include/LampController.h" "include/LowLevelController.h" "include/PalletizerController.h" "include/AngleManipulatorController.h" "include/UDPSocket.h")
set(SOURCES "src/LampController.cpp" "src/PalletizerController.cpp" "src/AngleManipulatorController.cpp" "src/UDPSocket.cpp")

set(FILES ${HEADERS} ${SOURCES})

add_library(lowlevelcontrollerlib STATIC ${FILES})