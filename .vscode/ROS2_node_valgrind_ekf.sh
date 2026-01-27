#!/bin/bash

source /opt/ros/humble/install/setup.bash
colcon build --packages-select advanced-ekf --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/local_setup.bash

rel_path=$(pwd)
rel_path+="/src/advanced-ekf/config/default.yaml"
ros2 run --prefix "valgrind --leak-check=full --error-limit=no --log-file=$(pwd)/valgrind.log" advanced-ekf advanced_odometry --ros-args --params-file ${rel_path} --log-level advanced_odometry_node:=DEBUG