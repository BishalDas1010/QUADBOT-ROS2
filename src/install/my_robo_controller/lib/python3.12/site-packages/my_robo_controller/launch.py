from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare("my_robo_controller"),
        "urdf",
        "my_roboBody.xacro"
    ])
    # Process the xacro to produce URDF
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", urdf_path
        ]),
        value_type=str
    )
      # Start Gazebo with empty world (Rolling uses newer Gazebo)
  # Start Gazebo with empty world (Rolling uses newer Gazebo)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', 'empty.sdf'],
        output='screen'
    )

        # Spawn robot in Gazebo (Updated package name for Rolling)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robo'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        gazebo,
        spawn_entity
    ])
