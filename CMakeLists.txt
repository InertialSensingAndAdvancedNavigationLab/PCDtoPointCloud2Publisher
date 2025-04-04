cmake_minimum_required(VERSION 3.8)
project(pcd_to_cloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include PCL directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create the executable
add_executable(PCDtoPointCloud2Publisher src/PCDtoPointCloud2Publisher.cpp)
ament_target_dependencies(PCDtoPointCloud2Publisher
rclcpp
sensor_msgs
PCL
pcl_conversions
)
target_link_libraries(PCDtoPointCloud2Publisher ${PCL_LIBRARIES})

# 链接库
install(TARGETS
  WinyunqPublisher
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch  # 安装launch目录
  DESTINATION share/${PROJECT_NAME}  # 安装到标准路径
)
# 安装配置文件
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
