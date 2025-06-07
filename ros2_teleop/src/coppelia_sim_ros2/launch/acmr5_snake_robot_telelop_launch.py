from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch CoppeliaSim (optional - you can start it manually)
        # ExecuteProcess(
        #     cmd=['coppeliaSim.sh', '-s', 'path/to/your/snake_scene.ttt'],
        #     output='screen'
        # ),
        
        # Launch the keyboard controller
        Node(
            package='coppelia_sim_ros2',
            executable='acmr5_controller',
            name='acmr5_controller',
            output='screen',
            emulate_tty=True,
            parameters=[]
        ),
    ])
