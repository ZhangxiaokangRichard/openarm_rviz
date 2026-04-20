cmake_minimum_required(VERSION 3.0.2)
project(fuel_gun_grasp)

## C++14 标准
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  open_manipulator_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs open_manipulator_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 主节点可执行文件
add_executable(fuel_gun_grasp_node src/fuel_gun_grasp_node.cpp)
target_link_libraries(fuel_gun_grasp_node ${catkin_LIBRARIES})
add_dependencies(fuel_gun_grasp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 安装规则
install(TARGETS fuel_gun_grasp_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Python 脚本安装（scene_marker_pub.py 等）
catkin_install_python(PROGRAMS
  scripts/scene_marker_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
