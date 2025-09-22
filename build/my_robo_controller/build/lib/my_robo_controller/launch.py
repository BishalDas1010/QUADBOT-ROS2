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
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', 'empty.sdf'],
        output='screen'
    )

    # Run the local cmd_vel publisher script so /cmd_vel is published
    # cmd_vel_script = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'cmd_vel_publisher.py'))
    # cmd_vel_node = ExecuteProcess(
    #     cmd=['python3', cmd_vel_script],
    #     output='screen'
    # )

    # Bridge ROS and Gazebo topics (Updated for Rolling)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry]',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model]',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V]'
        ],
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
    
    # # Joint State Publisher
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     output="screen"
    # )
    
    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )
    
    # Spawn robot in Gazebo with delay
    spawn_entity = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to fully load
        actions=[
            Node(
                package='ros_gz_sim',  # or 'gazebo_ros' for older versions
                executable='create',   # or 'spawn_entity.py' for older versions
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'my_robo'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        # cmd_vel_node,  # Commented out until script exists
        bridge,
        spawn_entity  # Fixed: removed duplicate TimerAction wrapper
    ])