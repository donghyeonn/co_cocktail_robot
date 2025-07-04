from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([


        Node(
                package='co_cocktail_robot',
                executable='detection',
                name='detection',
                output='screen'
        ),

        Node(
                package='co_cocktail_robot',
                executable='task_planning',
                name='task_planning',
                output='screen'
        ),
    ])