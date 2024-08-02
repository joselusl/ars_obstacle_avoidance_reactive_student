#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )

    ars_obstacle_avoidance_react_node_name_arg = DeclareLaunchArgument(
        'ars_obstacle_avoidance_react_node_name', default_value='ars_obstacle_avoidance_react_node',
        description='Name of the obstacle avoidance react node'
    )

    ars_obstacle_avoidance_react_yaml_file_arg=DeclareLaunchArgument(
      'config_param_obstacle_avoidance_react_yaml_file',
      default_value=PathJoinSubstitution(['config_obstacle_avoidance_react.yaml']), 
      description='Path to the config_param_obstacle_avoidance_react_yaml_file'
    )

    robot_cmd_stamped_topic_arg = DeclareLaunchArgument(
        'robot_cmd_stamped_topic', default_value='/robot_cmd_avoidance_stamped',
        description='Topic robot_cmd_stamped'
    )

    robot_cmd_raw_stamped_topic_arg = DeclareLaunchArgument(
        'robot_cmd_raw_stamped_topic', default_value='/robot_cmd_raw_stamped',
        description='Topic robot_cmd_raw_stamped'
    )

    obstacles_detected_topic_arg = DeclareLaunchArgument(
        'obstacles_detected_topic', default_value='/obstacles_detected_robot',
        description='Topic obstacles_detected'
    )

    # Get the launch configuration for parameters
    ars_obstacle_avoidance_react_conf_yaml_file = PathJoinSubstitution([FindPackageShare('ars_obstacle_avoidance_react'), 'config', LaunchConfiguration('config_param_obstacle_avoidance_react_yaml_file')])
    

    # Define the nodes
    ars_obstacle_avoidance_react_node = Node(
        package='ars_obstacle_avoidance_react',
        executable='ars_obstacle_avoidance_react_ros_node',
        name=LaunchConfiguration('ars_obstacle_avoidance_react_node_name'),
        output=LaunchConfiguration('screen'),
        parameters=[{'config_param_obstacle_avoidance_react_yaml_file': ars_obstacle_avoidance_react_conf_yaml_file}],
        remappings=[
          ('robot_cmd_raw_stamped', LaunchConfiguration('robot_cmd_raw_stamped_topic')),
          ('obstacles_detected', LaunchConfiguration('obstacles_detected_topic')),
          ('robot_cmd_avoidance_stamped', LaunchConfiguration('robot_cmd_stamped_topic')),
        ]
    )


    return LaunchDescription([
        screen_arg,
        ars_obstacle_avoidance_react_node_name_arg,
        ars_obstacle_avoidance_react_yaml_file_arg,
        robot_cmd_stamped_topic_arg,
        robot_cmd_raw_stamped_topic_arg,
        obstacles_detected_topic_arg,
        ars_obstacle_avoidance_react_node,
    ])

