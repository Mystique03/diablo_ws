# To launch robot in Gazebo with ros_control!

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os, xacro, yaml

def generate_launch_description():

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('diablo_bot'), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )])
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'diablo_robot',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.53',
                                   '-R', '0.0',
                                   '-P', '0.0',
                                   '-Y', '0.0',
                                   '-package_to_model',
                                   ],
                        parameters=[{'use_sim_time': True}],
                        output='screen')

    joint_state_broadcaster= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )
    
    right_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller"],
    )

    left_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller"],
    )

    right_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller"]
    )

    tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        )

    
    # Launch them all!
    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        joint_state_broadcaster,
        right_arm_controller,
        tf
    ])
