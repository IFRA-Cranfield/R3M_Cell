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

install(
  DIRECTORY 
    recipes
    initialconditions
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
  
  python/r3m_RecipeExecution.py
  python/Robot.py
  python/Gripper_Gz.py
  python/ObjectState.py
  python/ResetGazebo.py

  python/GzSim_Test.py
  python/m6d_test.py

  python/SpawnObject.py

  DESTINATION lib/${PROJECT_NAME} 
)

ament_package()