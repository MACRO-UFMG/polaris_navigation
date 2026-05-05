cmake_minimum_required(VERSION 3.8)
project(polaris_planning)


#-----------------------------------------------------------------------------
# 1. Compiler Configuration
#-----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

#-----------------------------------------------------------------------------
# 2. Dependency Management
#-----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

# Core ROS2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# Eigen3 configuration
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3 v${EIGEN3_VERSION}")

# Eigen compiler settings (applied globally)
add_compile_definitions(EIGEN_MPL2_ONLY EIGEN_NO_DEBUG)

# add_compile_options(-DEIGEN_NO_DEBUG)      # Disable debug asserts in Release

#-----------------------------------------------------------------------------
# 3. Executable Targets
#-----------------------------------------------------------------------------
# Define executables with their specific dependencies
add_executable(path_from_points src/path_from_points.cpp)
ament_target_dependencies(path_from_points
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
)
target_compile_features(path_from_points PRIVATE cxx_std_17)
target_include_directories(path_from_points PRIVATE ${EIGEN3_INCLUDE_DIR})


add_executable(path_from_file src/path_from_file.cpp)
ament_target_dependencies(path_from_file
  rclcpp
  geometry_msgs
  visualization_msgs
  std_srvs
  nav_msgs
)
target_compile_features(path_from_file PRIVATE cxx_std_17)
target_include_directories(path_from_file PRIVATE ${EIGEN3_INCLUDE_DIR})

add_executable(path_from_equation src/path_from_equation.cpp)
ament_target_dependencies(path_from_equation
  rclcpp
  geometry_msgs
  visualization_msgs
  std_srvs
  nav_msgs
)
target_compile_features(path_from_equation PRIVATE cxx_std_17)
target_include_directories(path_from_equation PRIVATE ${EIGEN3_INCLUDE_DIR})


#-----------------------------------------------------------------------------
# 4. Installation Configuration
#-----------------------------------------------------------------------------
# Install executables
install(TARGETS 
  path_from_points 
  path_from_file 
  path_from_equation
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(
  DIRECTORY 
    launch 
    config
  DESTINATION share/${PROJECT_NAME}
)

#-----------------------------------------------------------------------------
# 5. Package Export and Testing
#-----------------------------------------------------------------------------
# Export dependencies for downstream packages
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  eigen3_cmake_module
)

# Enable linting if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()