import os.path
import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('advanced-ekf')
    default_config_path = os.path.join(package_path, 'config')
    launch_folder_path = os.path.dirname(os.path.abspath(__file__))
    now = datetime.datetime.now()
    default_log_filename = 'ekf_log_' + now.strftime("%Y-%m-%d_%H-%M-%S") +  '.csv'
    default_log_filepath = os.path.join(launch_folder_path, '../../../../../src/advanced-ekf/logs')
    # default_rviz_config_path = os.path.join(
    #     package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')
    log_filepath = LaunchConfiguration('log_filepath')
    # rviz_use = LaunchConfiguration('rviz')
    # rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='default.yaml',
        description='Config file'
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='debug',
        description='Set the log level of advanced_odometry node'
    )
    declare_log_filepath_cmd = DeclareLaunchArgument(
        'log_filepath', default_value=os.path.join(default_log_filepath, default_log_filename),
        description='Set the log file path and filename of advanced_odometry node (default: src/advanced-ekf/logs)'
    )
    # declare_rviz_cmd = DeclareLaunchArgument(
    #     'rviz', default_value='true',
    #     description='Use RViz to monitor results'
    # )
    # declare_rviz_config_path_cmd = DeclareLaunchArgument(
    #     'rviz_cfg', default_value=default_rviz_config_path,
    #     description='RViz config file path'
    # )
    if not os.path.exists(default_log_filepath):
        os.makedirs(default_log_filepath)

    advanced_ekf_node = Node(
        package='advanced-ekf',
        executable='advanced_odometry',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time},
                    {'log_filepath': log_filepath}],
        output='screen',
        ros_arguments=["--log-level", ["advanced_odometry_node:=", log_level]]
    )
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_cfg],
    #     condition=IfCondition(rviz_use)
    # )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_log_filepath_cmd)
    # ld.add_action(declare_rviz_cmd)
    # ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(advanced_ekf_node)
    # ld.add_action(rviz_node)

    return ld
