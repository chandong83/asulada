cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_position_controller)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  /usr/local/include
)

add_executable(dynamixel_position_controller_node src/dynamixel_position_controller.cpp)
add_dependencies(dynamixel_position_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(": 32bit linux system")
    target_link_libraries(dynamixel_position_controller_node ${catkin_LIBRARIES} /usr/local/lib/libdxl_x86_cpp.so)
  elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(": 64bit linux system")
    target_link_libraries(dynamixel_position_controller_node ${catkin_LIBRARIES} /usr/local/lib/libdxl_x64_cpp.so)
  endif ()
else()
  message(": Non-linux system")
endif ()
