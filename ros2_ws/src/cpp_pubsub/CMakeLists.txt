cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


############################################ Resolve Package Dependencies ############################################

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)

# find_package(scipy REQUIRED)
# find_package(kinpy REQUIRED)

# find_package(builtin_interfaces REQUIRED)

find_package(tutorial_interfaces REQUIRED)   



############################################ CPP nodes ############################################

add_executable(position_talker src/position_talker.cpp)
ament_target_dependencies(position_talker rclcpp tutorial_interfaces)
target_link_libraries(position_talker /usr/local/lib/libdhd.so.3
                                      /usr/local/lib/libdhd.a
                                      # /usr/local/lib/libdhd.so.3.15.0
                                      # /usr/local/lib/libdrd.a
                                      /usr/local/lib/libdrd.so.3
                                      # /usr/local/lib/libdrd.so.3.15.0
)

add_executable(gazebo_controller src/gazebo_controller.cpp)
ament_target_dependencies(gazebo_controller rclcpp tutorial_interfaces std_msgs trajectory_msgs sensor_msgs kdl_parser)

# add_executable(real_controller src/real_controller.cpp)
# ament_target_dependencies(real_controller rclcpp tutorial_interfaces std_msgs trajectory_msgs sensor_msgs kdl_parser)

add_executable(const_br src/const_br.cpp)
ament_target_dependencies(const_br geometry_msgs rclcpp tf2 tf2_ros angles)

add_executable(marker_publisher src/marker_publisher.cpp)
ament_target_dependencies(marker_publisher rclcpp tutorial_interfaces geometry_msgs visualization_msgs)


install(TARGETS

  gazebo_controller

  position_talker
  # real_controller
  const_br
  marker_publisher
  
  DESTINATION lib/${PROJECT_NAME}
)





############################################ Python nodes ############################################

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS

  scripts/traj_recorder.py
  scripts/fitts_task.py

  DESTINATION lib/${PROJECT_NAME}
)


############################################ Launch files ############################################

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)




############################################ Build Testing Steps ############################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
