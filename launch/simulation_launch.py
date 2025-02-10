from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    number_of_sim = 1

    ld = LaunchDescription()
    controller_node = Node(
        package= 'ros2_python_tutorial',
        namespace= 'turtlesim1',
        executable= 'turtlebot_controller',
        output='screen'
    )
    ld.add_action(controller_node)

    simulator_node = Node(
        package='ros2_python_tutorial',
        namespace = 'turtlesim1',
        executable='turtlebot_simulator',
        output='screen'
    )
    ld.add_action(simulator_node)

    rviz_node = Node(
        package='rviz2',
        namespace='rviz2',
        executable='rviz2',
        name='rviz2',
    )
    ld.add_action(rviz_node)

    return ld