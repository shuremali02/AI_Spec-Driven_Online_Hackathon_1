from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file to start multiple robot instances with different configurations."""

    # Declare launch arguments
    robot1_name = DeclareLaunchArgument(
        'robot1_name',
        default_value='robot_alpha',
        description='Name of the first robot'
    )

    robot2_name = DeclareLaunchArgument(
        'robot2_name',
        default_value='robot_beta',
        description='Name of the second robot'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Robot 1 nodes
    robot1_controller = Node(
        package='robot_controller',
        executable='controller_node',
        name='controller',
        namespace=LaunchConfiguration('robot1_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_name': LaunchConfiguration('robot1_name')},
            {'max_velocity': 1.0}
        ],
        output='screen'
    )

    robot1_sensor = Node(
        package='sensor_package',
        executable='lidar_node',
        name='lidar',
        namespace=LaunchConfiguration('robot1_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Robot 2 nodes
    robot2_controller = Node(
        package='robot_controller',
        executable='controller_node',
        name='controller',
        namespace=LaunchConfiguration('robot2_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_name': LaunchConfiguration('robot2_name')},
            {'max_velocity': 1.5}  # Different max velocity for robot2
        ],
        output='screen'
    )

    robot2_sensor = Node(
        package='sensor_package',
        executable='lidar_node',
        name='lidar',
        namespace=LaunchConfiguration('robot2_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Visualization node to monitor both robots
    viz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='multi_robot_viz',
        arguments=['-d', 'path/to/multi_robot_config.rviz'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    return LaunchDescription([
        robot1_name,
        robot2_name,
        use_sim_time,
        robot1_controller,
        robot1_sensor,
        robot2_controller,
        robot2_sensor,
        viz_node
    ])