
cmake_minimum_required(VERSION 3.5)
project(example_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

######################################################

# Install Python Module only once
if(NOT TARGET ament_cmake_python_copy_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()

# Install Python executables
install(PROGRAMS
  scripts/dummy_script.py
  scripts/controller.py
  scripts/jointstate_script.py
  scripts/workspace.py
  scripts/random_target.py
 
  scripts/Teleop_mode.py
  scripts/teleop_key.py




  DESTINATION lib/${PROJECT_NAME}
)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node_test src/cpp_node.cpp)
ament_target_dependencies(cpp_node_test rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_node_test
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  config
  launch
  meshes
  robot
  scripts
  
  DESTINATION share/${PROJECT_NAME}
)



# Finalize the package
ament_package()




