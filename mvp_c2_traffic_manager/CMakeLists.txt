cmake_minimum_required(VERSION 3.5.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(mvp_c2_traffic_manager)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/")

## Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(goby 3.1 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${CMAKE_BINARY_DIR})

####################################################################
# build                      
####################################################################

set(ros_lib
  rclcpp
  rclcpp_components
  ament_index_cpp
  std_msgs 
)

set(other_lib
  goby
  yaml-cpp
)

set(executable_name mvp_c2_traffic_manager)

add_executable(${executable_name}
  src/mvp_c2_traffic_manager.cpp
  src/node.cpp
)

## Corrected dependencies for ROS 2 
ament_target_dependencies(${executable_name} ${ros_lib})
target_link_libraries(${executable_name} ${other_lib})


####################################################################
# install                      
####################################################################
# Install
install(TARGETS ${library_name} ${executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_dependencies(${ros_lib})
ament_package()
