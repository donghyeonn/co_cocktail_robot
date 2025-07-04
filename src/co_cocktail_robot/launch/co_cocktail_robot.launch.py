from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        ExecuteProcess(
            cmd=['ros2', 'launch', 'dsr_bringup2', 'dsr_bringup2_rviz.launch.py',
                'mode:=real', 'host:=192.168.1.100', 'port:=12345', 'model:=m0609'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'launch', 'realsense2_camera', 'rs_align_depth_launch.py',
                'depth_module.depth_profile:=640x480x30',
                'rgb_camera.color_profile:=640x480x30',
                'initial_reset:=true',
                'align_depth.enable:=true',
                'enable_rgbd:=true',],
            output='screen'
        ),

        Node(
            package='co_cocktail_robot',
            executable='main',
            name='main',
            output='screen'
        ),
    ])