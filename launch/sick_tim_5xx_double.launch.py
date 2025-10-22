from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    laser_filter = os.path.join(get_package_share_directory('sick_scan_xd'), 'config', 'laser_filter.yaml')
    ld = LaunchDescription()

    scanner1_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim_5xx_1',
        output='screen',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'frame_id': 'lidar_1',
            'hostname': '192.168.0.1',
            'cloud_topic': 'cloud_1',
            'port': '2112',
            'min_ang': -2.35619449,  # -135 deg
            'max_ang': 2.35619449,   # 135 deg
            'tf_publish_rate': 0.0 ,
            'range_min': 0.15,
            'range_max': 10.0,
            'publish_tf': False,
            'range_filter_handling': 2,
            # 'filter_settings': 1,
            # 'intensity': True, 
            # 'reflectivity_threshold': 100,
            # 'contour_reduction': 1,
            # 'median_filter': 3, 
            # 'averaging_filter': 2,
            # 'range_min': 0.1, 
            # 'range_max': 10.0
        }],
        remappings=[
            ('sick_tim_5xx/scan', 'scan_1'),
            ('cloud', 'cloud_1')
        ]
    )

    scanner2_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim_5xx_2',
        output='screen',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'frame_id': 'lidar_2',
            'hostname': '192.168.0.2',
            'cloud_topic': 'cloud_2',
            'port': '2112',
            'min_ang': -2.35619449,  # -135 deg
            'max_ang': 2.35619449,   # 135 deg
            'tf_publish_rate': 0.0 ,
            'range_min': 0.15,
            'range_max': 10.0,
            'publish_tf': False,
            'range_filter_handling': 2,
            # 'filter_settings': 1,
            # 'intensity': True,
            # 'reflectivity_threshold': 100,
            # 'contour_reduction': 1,
            # 'median_filter': 3,
            # 'averaging_filter': 2,
            # 'range_min': 0.1,
            # 'range_max': 10.0
            
        }],
        remappings=[
            ('sick_tim_5xx/scan', 'scan_2'),
            ('cloud', 'cloud_2')
        ]
    )

    scan_filter_node=Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_node',
            output='screen',
            parameters=[{"scan_topic": "/merged_laserscan"},
                {"filtered_scan_topic": "/filtered_scan"},
                laser_filter],
            remappings=[
                ('scan', '/merged_laserscan') # original scan input
            ]
        )
    
    lidar_merger = ExecuteProcess(
        cmd=['python3', '/home/awc/sick_scan_ws/src/sick_scan_xd/scripts/lidar_merger.py'],
        output='screen',
        name='lidar_merger'
    )
    
    ld.add_action(scanner1_node)
    ld.add_action(scanner2_node)
    ld.add_action(lidar_merger)
    #ld.add_action(scan_filter_node)

    return ld
