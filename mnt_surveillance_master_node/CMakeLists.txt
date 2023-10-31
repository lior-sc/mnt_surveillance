###############################################################################
# Set minimum required version of cmake, project name and compile options
###############################################################################
cmake_minimum_required(VERSION 3.8)
project(mnt_surveillance_master_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################
# find dependencies
###############################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)


###############################################################################
# build
###############################################################################
include_directories(
  include
  )

add_library(${PROJECT_NAME}_lib
  src/master_node.cpp
  src/alarm.cpp
  src/recorder.cpp
  src/analyzers/analyzer.cpp
)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

target_link_libraries(${PROJECT_NAME}_lib)                      # link libraries to project
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})  # link ros libraries to target

# create executable and link libraries and ament packages
set(EXECUTABLE_NAME master_node)
add_executable(${EXECUTABLE_NAME} src/node_main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

################################################################################
# Install
################################################################################
# install(DIRECTORY param
#   DESTINATION share/${PROJECT_NAME}
# )

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
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


