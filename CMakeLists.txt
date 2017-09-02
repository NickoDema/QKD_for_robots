cmake_minimum_required(VERSION 2.8.3)
project(kvant)

## c++17 standart
## add_compile_options(-std=gnu++1z)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
 FILES
 CryptString.msg
)

add_service_files(
FILES
Set_key.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kvant
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable( slave
  src/kvant_slave_node.cpp
  src/kvant_slave.cpp
  src/kvant_basic.cpp
)

target_link_libraries(slave
  ${catkin_LIBRARIES}
)

## add_executable( master
##  src/kvant_master_node.cpp
##  src/kvant_master.cpp
##)

##target_link_libraries(master
##  ${catkin_LIBRARIES}
##)
