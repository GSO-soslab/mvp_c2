cmake_minimum_required(VERSION 3.8)

project(mvp_c2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(INTERFACE_FILES "")

##################################################
# msg
##################################################
file(GLOB MSG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/**/*.msg)
foreach(I IN ITEMS ${MSG_FILES})
  get_filename_component(ITEM_NAME ${I} NAME)
  list(APPEND INTERFACE_FILES msg/${ITEM_NAME})
endforeach()

##################################################
# srv
##################################################
# file(GLOB SRV_FILES ${CMAKE_CURRENT_SOURCE_DIR}/**/*.srv)
# foreach(I IN ITEMS ${SRV_FILES})
#   get_filename_component(ITEM_NAME ${I} NAME)
#   list(APPEND INTERFACE_FILES srv/${ITEM_NAME})
# endforeach()

##################################################
# action
##################################################
# file(GLOB ACT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/**/*.action)
# foreach(I IN ITEMS ${ACT_FILES})
#   get_filename_component(ITEM_NAME ${I} NAME)
#   list(APPEND INTERFACE_FILES action/${ITEM_NAME})
# endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACE_FILES}
  DEPENDENCIES
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

