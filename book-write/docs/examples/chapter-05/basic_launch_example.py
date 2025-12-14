from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file to start 3 basic nodes: talker, listener, and parameter node."""

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # First node: publisher
    talker_node = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Second node: subscriber
    listener_node = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='listener_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Third node: parameter demo node
    parameter_node = Node(
        package='demo_nodes_cpp',
        executable='parameter_node',
        name='param_demo_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'param1': 'value1'},
            {'param2': 42}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        talker_node,
        listener_node,
        parameter_node
    ])