import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

package_name = 'ba_description'

def generate_launch_description():
                        
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'urdf','braccio.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml()}

    rviz_path = os.path.join(get_package_share_directory(package_name), 'rviz', 'braccio_arm_rviz.rviz')
          
    return LaunchDescription(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='RSPublisher',
                parameters=[params]
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='JSPublisher'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='RViz2',
                output='screen',
                arguments=['-d'+ rviz_path]
            )
        ]
    )