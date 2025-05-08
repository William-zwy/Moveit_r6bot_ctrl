cmake_minimum_required(VERSION 3.8)
project(ctrl_rbot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)

# find dependencies
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
    ctrl_rbot
    SHARED
    hardware/r6bot_hardware.cpp
  )

add_executable(ctrl_rbot_node 
  src/ctrl_rbot.cpp
)

target_include_directories(ctrl_rbot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(
  ctrl_rbot_node
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

install(TARGETS ctrl_rbot_node
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(ctrl_rbot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ctrl_rbot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ctrl_rbot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
)
ament_target_dependencies(
  ctrl_rbot
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
  moveit_ros_planning_interface
  rclcpp
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ctrl_r6bot.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface ctrl_r6bot.xml)

install(TARGETS ctrl_rbot
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf 
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY hardware/include/ 
  DESTINATION share/${PROJECT_NAME})

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
