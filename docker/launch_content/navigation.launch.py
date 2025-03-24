import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    drive_params = PathJoinSubstitution([
      '.',
      'edu_drive_edu_bot.yaml'
    ])

    lidar_params = PathJoinSubstitution([
      '.',
      'TminiPro.yaml'
    ])
    
    nav2_params = PathJoinSubstitution([
      '.',
      'nav2_params.yaml'
    ])

    edu_drive = Node(
      package='edu_drive_ros2',
      executable='edu_drive_ros2_node',
      name='edu_drive_ros2_node',
      parameters=[drive_params],
      namespace='', #os.environ.get('EDU_ROBOT_NAMESPACE', "eduard"),
      output='screen'
    )  

    lidar_node = Node(package='ydlidar_ros2_driver',
      executable='ydlidar_ros2_driver_node',
      name='ydlidar_ros2_driver_node',
      output='screen',
      emulate_tty=True,
      parameters=[lidar_params],
      namespace=''
    )

    lidar_tf = Node(package='tf2_ros',
      executable='static_transform_publisher',
      name='base_link_to_base_laser',
      arguments=["--x", "0.115","--y", "0.0","--z", "0.15","--roll",  "0","--pitch", "0.0","--yaw",  "1.57", "--frame-id", "base_link","--child-frame-id", "laser_link"],
      namespace=''
    )
    
    csm_node = Node(package='ros2_laser_scan_matcher',
      executable='laser_scan_matcher',
      name='laser_scan_matcher',
      output='screen',
      emulate_tty=True,
      parameters=[{
          'publish_tf':   True,
          'base_frame':   'base_link',
          'odom_frame':   'odom',
          'map_frame':    'map',
          'laser_frame':  'laser_link'
        }],
      namespace=''
    )
    
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            './',
            'slam_toolbox.launch.py'
          ])
        ]),

      launch_arguments={
        'use_sim_time': 'false'
      }.items()
    )

    nav2_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        ])
      ]),

      launch_arguments={
        'params_file': nav2_params,
        'use_sim_time': 'false'
      }.items(),
    )

    return LaunchDescription([
        #edu_drive,
        lidar_node,
        lidar_tf,
        csm_node, # edu_drive_ros2 also publishes odometry data. Make sure you only have one active odometry source at a time.
        slam_toolbox_node,
        nav2_node
    ])