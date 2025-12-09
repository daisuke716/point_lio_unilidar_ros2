from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_dir = get_package_share_directory('point_lio_unilidar')

    config_file = PathJoinSubstitution([
        pkg_dir,
        'config',
        'unilidar_l1.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        pkg_dir,
        'rviz_cfg',
        'loam_livox.rviz'
    ])

    lio_node = Node(
        package='point_lio_unilidar',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            config_file,    # 自动加载 YAML
            {'use_imu_as_input': False},
            {'prop_at_freq_of_imu': True},
            {'check_satu': True},
            {'init_map_size': 10},
            {'point_filter_num': 1},
            {'space_down_sample': True},
            {'filter_size_surf': 0.1},
            {'filter_size_map': 0.1},
            {'cube_side_length': 1000.0},
            {'runtime_pos_log_enable': False},
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        lio_node,
        rviz_node
    ])
