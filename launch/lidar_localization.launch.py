import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from launch_param_overrides import resolve_parameter_overrides  # noqa: E402

# Node parameters that a launch argument overrides only when it is set.
# Empty arguments keep the parameter YAML value, or the fallback below.
PARAMETER_OVERRIDES = {
    'use_sim_time': (bool, False),
    'global_frame_id': (str, 'map'),
    'odom_frame_id': (str, 'odom'),
    'base_frame_id': (str, 'base_link'),
    'use_imu_preintegration': (bool, True),
    'imu_preintegration_use_base_frame_transform': (bool, False),
    'enable_map_odom_tf': (bool, False),
    'use_odom': (bool, False),
    'use_odom_tf_prediction': (bool, False),
    'publish_bridge_pose_when_lost': (bool, False),
    'use_continuous_time_deskew': (bool, True),
    'continuous_time_deskew_reference_time_sec': (float, 0.0),
}


def generate_launch_description():
    default_localization_param_dir = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'param',
        'localization.yaml')
    localization_param_dir = LaunchConfiguration('localization_param_dir')
    cloud_topic = LaunchConfiguration('cloud_topic')
    twist_topic = LaunchConfiguration('twist_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    base_frame_id = LaunchConfiguration('resolved_base_frame_id')
    use_dataset_tf_tree = LaunchConfiguration('use_dataset_tf_tree')
    dataset_root_frame = LaunchConfiguration('dataset_root_frame')
    publish_lidar_tf = LaunchConfiguration('publish_lidar_tf')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    lidar_tf_x = LaunchConfiguration('lidar_tf_x')
    lidar_tf_y = LaunchConfiguration('lidar_tf_y')
    lidar_tf_z = LaunchConfiguration('lidar_tf_z')
    lidar_tf_roll = LaunchConfiguration('lidar_tf_roll')
    lidar_tf_pitch = LaunchConfiguration('lidar_tf_pitch')
    lidar_tf_yaw = LaunchConfiguration('lidar_tf_yaw')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    imu_tf_x = LaunchConfiguration('imu_tf_x')
    imu_tf_y = LaunchConfiguration('imu_tf_y')
    imu_tf_z = LaunchConfiguration('imu_tf_z')
    imu_tf_roll = LaunchConfiguration('imu_tf_roll')
    imu_tf_pitch = LaunchConfiguration('imu_tf_pitch')
    imu_tf_yaw = LaunchConfiguration('imu_tf_yaw')

    # Default robot: base frame -> lidar. Bagged datasets (e.g. Koide Zenodo 10122133): set
    # use_dataset_tf_tree:=true and dataset_root_frame:=camera_base so the base frame attaches to
    # the bag's /tf_static tree (depth_camera_link, imu_link, ...).
    lidar_tf = Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', lidar_tf_x,
            '--y', lidar_tf_y,
            '--z', lidar_tf_z,
            '--roll', lidar_tf_roll,
            '--pitch', lidar_tf_pitch,
            '--yaw', lidar_tf_yaw,
            '--frame-id', base_frame_id,
            '--child-frame-id', lidar_frame_id,
        ],
        condition=IfCondition(PythonExpression([
            "'", publish_lidar_tf, "' == 'true' and '", use_dataset_tf_tree, "' != 'true'"
        ])))

    imu_tf = Node(
        name='imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', imu_tf_x,
            '--y', imu_tf_y,
            '--z', imu_tf_z,
            '--roll', imu_tf_roll,
            '--pitch', imu_tf_pitch,
            '--yaw', imu_tf_yaw,
            '--frame-id', base_frame_id,
            '--child-frame-id', imu_frame_id,
        ],
        condition=IfCondition(publish_imu_tf))

    dataset_root_attach_tf = Node(
        name='dataset_root_attach',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', base_frame_id,
            '--child-frame-id', dataset_root_frame,
        ],
        condition=IfCondition(use_dataset_tf_tree))

    # Extrinsics from Koide indoor_* bags (Zenodo 10122133), duplicated here because /tf_static
    # from ros2 bag play often fails QoS matching with transform listeners.
    koide_camera_base_to_depth_tf = Node(
        name='koide_camera_base_to_depth',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'camera_base',
            '--child-frame-id', 'depth_camera_link',
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0017999999690800905',
            '--qx', '0.5254827454987588',
            '--qy', '-0.5254827454987588',
            '--qz', '0.473146789255815',
            '--qw', '-0.4731467892558148',
        ],
        condition=IfCondition(use_dataset_tf_tree))

    koide_depth_to_imu_tf = Node(
        name='koide_depth_to_imu',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'depth_camera_link',
            '--child-frame-id', 'imu_link',
            '--x', '0.003463566434548747',
            '--y', '0.0041740033449125195',
            '--z', '-0.05071645628165228',
            '--qx', '-0.47551892422054987',
            '--qy', '0.4736570557366866',
            '--qz', '0.5236230430890362',
            '--qw', '0.5247376978138559',
        ],
        condition=IfCondition(use_dataset_tf_tree))

    lidar_localization = Node(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[
            localization_param_dir,
            LaunchConfiguration('lidar_localization_override_file'),
        ],
        remappings=[
            ('/cloud', cloud_topic),
            ('/twist', twist_topic),
            ('/imu', imu_topic),
            ('/odom', odom_topic),
        ],
        output='screen')

    startup = Node(
        package='lidar_localization_ros2',
        executable='start_lifecycle_node.py',
        arguments=['lidar_localization'],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization_param_dir',
            default_value=default_localization_param_dir,
            description='Path to the lidar_localization_ros2 parameter YAML.'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/velodyne_points',
            description='Input sensor_msgs/PointCloud2 topic remapped to /cloud.'),
        DeclareLaunchArgument(
            'twist_topic',
            default_value='/twist',
            description='Optional twist topic remapped to /twist.'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu',
            description='Optional IMU topic remapped to /imu.'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Optional nav_msgs/Odometry seed topic remapped to /odom '
                        '(used as a twist-integration fallback when IMU '
                        'preintegration is off, or as the source TF for '
                        'enable_map_odom_tf).'),
        DeclareLaunchArgument(
            'global_frame_id',
            default_value='',
            description='Empty keeps the parameter YAML value (fallback: map).'),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='',
            description='Empty keeps the parameter YAML value (fallback: odom).'),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='',
            description='Empty keeps the parameter YAML value (fallback: base_link).'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='',
            description='Empty keeps the parameter YAML value (fallback: false).'),
        DeclareLaunchArgument(
            'use_imu_preintegration',
            default_value='',
            description='Empty keeps the parameter YAML value (fallback: true).'),
        DeclareLaunchArgument(
            'imu_preintegration_use_base_frame_transform',
            default_value='',
            description='Empty keeps the parameter YAML value (fallback: false).'),
        DeclareLaunchArgument(
            'enable_map_odom_tf',
            default_value='',
            description='Look up an external odom -> base_frame_id TF (e.g. from an '
                        'external LIO front end) and publish map -> odom instead of '
                        'map -> base_frame_id directly.'
                        ' Empty keeps the parameter YAML value (fallback: false).'),
        DeclareLaunchArgument(
            'use_odom',
            default_value='',
            description='Subscribe odom_topic (nav_msgs/Odometry) as a twist-'
                        'integration seed fallback when IMU preintegration is off.'
                        ' Empty keeps the parameter YAML value (fallback: false).'),
        DeclareLaunchArgument(
            'use_odom_tf_prediction',
            default_value='',
            description='Seed scan registration from the composed frozen '
                        'map -> odom x live odom -> base_frame_id TF (external '
                        'LIO front end) instead of internal dead reckoning; '
                        'requires enable_map_odom_tf.'
                        ' Empty keeps the parameter YAML value (fallback: false).'),
        DeclareLaunchArgument(
            'publish_bridge_pose_when_lost',
            default_value='',
            description='While scan matching is rejected, keep publishing the '
                        'odom-bridge composed pose as the pose output so the '
                        'estimate stays continuous through dropouts; requires '
                        'enable_map_odom_tf.'
                        ' Empty keeps the parameter YAML value (fallback: false).'),
        DeclareLaunchArgument(
            'use_continuous_time_deskew',
            default_value='',
            description='Deskew scans when point timing and motion data are ready; '
                        'otherwise keep the input scan unchanged.'
                        ' Empty keeps the parameter YAML value (fallback: true).'),
        DeclareLaunchArgument(
            'continuous_time_deskew_reference_time_sec',
            default_value='',
            description='Empty keeps the parameter YAML value (fallback: 0.0).'),
        DeclareLaunchArgument(
            'publish_lidar_tf',
            default_value='true',
            description='Publish a static base_frame_id -> lidar_frame_id transform.'),
        DeclareLaunchArgument('lidar_frame_id', default_value='velodyne'),
        DeclareLaunchArgument('lidar_tf_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'publish_imu_tf',
            default_value='false',
            description='Publish a static base_frame_id -> imu_frame_id transform.'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('imu_tf_x', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_y', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_z', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('imu_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'use_dataset_tf_tree',
            default_value='false',
            description='Attach base_frame_id to a dataset-provided TF tree instead of lidar TF.'),
        DeclareLaunchArgument('dataset_root_frame', default_value='camera_base'),
        resolve_parameter_overrides(
            'localization_param_dir', 'lidar_localization', PARAMETER_OVERRIDES),
        lidar_localization,
        lidar_tf,
        imu_tf,
        dataset_root_attach_tf,
        koide_camera_base_to_depth_tf,
        koide_depth_to_imu_tf,
        startup,
    ])
