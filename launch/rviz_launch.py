import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'rviz_transformations'
    urdf_file = os.path.join(
        os.path.expanduser('~/mocap_ws/src'), pkg_name, 'urdf', 'drone2.urdf'
    )
    rviz_config_file = os.path.join(
        os.path.expanduser('~/mocap_ws/src'), pkg_name, 'rvizconfig', 'pleasedontbreak.rviz'
    )

    return LaunchDescription([
        #Publish the robot description (this will publish the static stsructure from the URDF)
        # Node(   
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     arguments=[urdf_file],
        #     output='screen'
        # ),
        #Start RViz2 with your configuration
                # Launch your external TF broadcaster node
        Node(
            package=pkg_name,
            executable='externalnodepublish',
            name='nodepublisher',
            output='screen'
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.3', '0', '-1.2', '0', '0', '-0.7071068', '0.7071068', 'tiramisu', 'tiramisubaselink'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0', '-0.5', '0', '0', '-0.0', '1', 'bunker_octo', 'bunker_mini'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.2', '0', '-0.2', '0', '0', '1', '0', 'Telloextra', 'Tellobase'],
            output='screen'
        ),

        
        #ros2 run tf2_ros static_transform_publisher 0 0 -1 0 0 0 1 tiramisu tiramisubaselink


        
    ])

