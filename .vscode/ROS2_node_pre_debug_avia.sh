#!/bin/bash

  source /opt/ros/humble/install/setup.bash
  # colcon build --packages-select slam_tools --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  colcon build --packages-select fast_lio --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  source install/local_setup.bash

  # ros2 run --prefix 'gdbserver localhost:3000' slam_tools pointcloud2las
  filepath=$(echo pointcloud_$(date +"%Y_%m_%d-%H_%M_%S").pcd)
  # ros2 run --prefix 'gdbserver localhost:3000' slam_tools pointcloud2pcd -p "pcd_output_path:=${filepath}"
  rel_path=$(pwd)
  rel_path+="/src/fast-lio/config/avia.yaml"
  ros2 run --prefix 'gdbserver localhost:3000' fast_lio fastlio_mapping --ros-args --params-file ${rel_path} #--log-level DEBUG

