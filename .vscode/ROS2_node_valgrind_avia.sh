#!/bin/bash

source /opt/ros/humble/install/setup.bash
colcon build --packages-select fast_lio --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/local_setup.bash

rel_path=$(pwd)
rel_path+="/src/fast-lio/config/avia.yaml"
valgrind_prefix="valgrind --leak-check=full --error-limit=no --log-file=$(pwd)/valgrind.log"
ros2 run --prefix "valgrind --leak-check=full --error-limit=no --log-file=$(pwd)/valgrind.log" fast_lio fastlio_mapping --ros-args --params-file ${rel_path} #--log-level DEBUG

