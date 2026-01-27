cmake_minimum_required(VERSION 3.8)
project(advanced-ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

# if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
#   include(ProcessorCount)
#   ProcessorCount(N)
#   message("Processer number:  ${N}")

#   if(N GREATER 4)
#     add_definitions(-DMP_EN)
#     add_definitions(-DMP_PROC_NUM=3)
#     message("core for MP: 3")
#   elseif(N GREATER 3)
#     add_definitions(-DMP_EN)
#     add_definitions(-DMP_PROC_NUM=2)
#     message("core for MP: 2")
#   else()
#     add_definitions(-DMP_PROC_NUM=1)
#   endif()
# else()
#   add_definitions(-DMP_PROC_NUM=1)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen found at: ${EIGEN3_INCLUDE_DIR}")
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
message(STATUS "OpenMP flags: ${OpenMP_CXX_FLAGS}; CXX flags: ${CMAKE_CXX_FLAGS}")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include/advanced-ekf)
include_directories(include/IKFoM/IKFoM_toolkit)

add_executable(advanced_odometry src/advanced_odometry.cpp src/ekf.cpp)
target_include_directories(advanced_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(advanced_odometry PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(advanced_odometry rclcpp sensor_msgs nav_msgs geometry_msgs px4_msgs)
target_link_libraries(advanced_odometry ${EIGEN_LIBRARIES})

install(TARGETS advanced_odometry
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
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
