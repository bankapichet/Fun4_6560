from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
import xacro    
    
def generate_launch_description():
    
    pkg = get_package_share_directory('example_description')
    
    
    rviz_path = os.path.join(pkg, 'config', 'display.rviz')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen'
    )
    
    path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    parameters = [{'robot_description': robot_desc_xml}]
    
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )
    
    teleop_node = Node(
        package='example_description',  
        executable='Teleop_mode.py',  
        output='screen'
    )
    
    
    teleopkey_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'example_description', 'teleop_key.py'],
        output='screen'
    )
    
    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(teleop_node)
    launch_description.add_action(teleopkey_node)

    return launch_description
