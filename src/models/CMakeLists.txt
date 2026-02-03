cmake_minimum_required(VERSION 3.22)
project(models)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置依赖库的搜索路径
set(console_bridge_DIR "/usr/lib/x86_64-linux-gnu/console_bridge/cmake")
set(urdfdom_DIR "/usr/lib/x86_64-linux-gnu/urdfdom/cmake")
set(urdfdom_headers_DIR "/usr/share/urdfdom_headers/cmake")
set(TINYXML2_LIBRARY "/usr/lib/x86_64-linux-gnu/libtinyxml2.so")
set(TINYXML2_INCLUDE_DIR "/usr/include")

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()