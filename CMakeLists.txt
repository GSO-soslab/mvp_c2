cmake_minimum_required(VERSION 3.5)
project(mvp_c2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install the entry point for the executable
install(
    PROGRAMS
    scripts/mvp_c2_reporter.py  # Adjust the path if needed
    DESTINATION lib/${PROJECT_NAME}
)

# Install config folder
install(
    DIRECTORY proto/
    DESTINATION share/${PROJECT_NAME}/proto
)
ament_package()
