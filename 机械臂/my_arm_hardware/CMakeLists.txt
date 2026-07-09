cmake_minimum_required(VERSION 3.8)
project(my_arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 将我们的源文件编译成一个名为 my_arm_hardware 的共享动态链接库 (.so)
add_library(my_arm_hardware SHARED
  src/socketcan_interface.cpp
)

# 告诉编译器去哪里找刚刚写的头文件
target_include_directories(my_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接刚才找到的 ROS 2 依赖包
ament_target_dependencies(my_arm_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# 极重要的一步：把我们写的 xml 文件导出去，让系统知道这是一个插件包
pluginlib_export_plugin_description_file(hardware_interface my_arm_hardware.xml)

# 安装规则：编译完成后，把动态库和头文件放到系统指定的安装目录下
install(TARGETS my_arm_hardware
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES controllers.yaml
  DESTINATION share/${PROJECT_NAME}
)


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
