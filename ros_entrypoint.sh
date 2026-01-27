#!/bin/sh

echo "source '/opt/ros/$ROS_DISTRO/install/setup.bash'" >> ~/.bashrc
echo "source '/workspaces/advanced-ekf/install/local_setup.bash'" >> ~/.bashrc

exec "$@"
