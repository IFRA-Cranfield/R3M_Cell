cmake_minimum_required(VERSION 3.8)
project(r3mcell_execution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# =========================================================== #
# REQUIRED TO EXECUTE .cpp scripts -> r3m_wrapper.cpp:

# REQUIRED to -> Launch move_group interface:
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# REQUIRED to -> Include r3mcell_data package:
find_package(r3mcell_data REQUIRED)

# REQUIRED to -> Include linkattacher_msgs package:
find_package(linkattacher_msgs REQUIRED)

# REQUIRED to -> Include objectpose_msgs package:
find_package(objectpose_msgs REQUIRED)

# REQUIRED to -> Include objectpose_msgs package:
find_package(linkpose_msgs REQUIRED)

# REQUIRED to -> Execute ACTIONS:
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add executable:
add_executable(
  r3m_wrapper
  src/r3m_wrapper.cpp
)

include_directories(${YAML_CPP_INCLUDE_DIR})
target_link_libraries(r3m_wrapper ${YAML_CPP_LIBRARIES})

# Install executable:
install(TARGETS
  r3m_wrapper
  DESTINATION lib/${PROJECT_NAME}
)

# Add ament dependencies:
ament_target_dependencies(
  r3m_wrapper
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
  rclcpp_action
  r3mcell_data
  linkattacher_msgs
  objectpose_msgs
  linkpose_msgs
  yaml-cpp
  ament_index_cpp
)

install(
  DIRECTORY 
    recipes
  DESTINATION 
    share/${PROJECT_NAME}
)

# =========================================================== #
# REQUIRED TO EXECUTE .py scripts:
install(
  DIRECTORY 
    python
  DESTINATION 
    share/${PROJECT_NAME}
)
install(PROGRAMS
  python/SpawnObject.py
  python/r3m_matlab.py
  DESTINATION lib/${PROJECT_NAME} 
)

ament_package()