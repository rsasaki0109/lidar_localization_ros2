"""Three-node bringup for the G3 guarded automatic reinitialization loop.

Brings up, in one launch, the full closed recovery loop documented in
``docs/global_localization_roadmap.md`` (G3 section):

1. the core ``lidar_localization`` lifecycle node (raises
   ``/reinitialization_requested`` and publishes ``/alignment_status``),
2. the G2 ``global_localization_node`` (answers ``~/query`` with ranked
   candidates from a map-wide BBS_2D search),
3. the G3 ``reinitialization_supervisor_node`` (watches the lost-tracking
   signal, queries G2, and republishes ``/initialpose`` -- only when every
   safety guard in ``reinitialization_supervisor_policy`` passes).

This is the bringup the roadmap's post-reset recovery-evidence gate needs: with
all three running against the Koide kidnapped-start window, tracking should
actually recover after the supervisor publishes its guarded reset. Everything is
still opt-in -- this launch is never part of the default bringup, and the
supervisor publishes nothing until ``/reinitialization_requested`` is asserted
and the guards pass.

Example (Koide outdoor_hard_01a, dataset TF tree)::

    ros2 launch lidar_localization_ros2 global_localization_recovery.launch.py \\
        cloud_topic:=/velodyne_points \\
        occupancy_yaml:=/path/to/occupancy.yaml \\
        localization_param_dir:=/path/to/localization.yaml \\
        use_dataset_tf_tree:=true dataset_root_frame:=camera_base \\
        use_sim_time:=true

Then replay the bag with ``set_initial_pose:=false`` (or let tracking diverge);
the supervisor closes the loop without manual intervention.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_localization_ros2')

    cloud_topic = LaunchConfiguration('cloud_topic')
    occupancy_yaml = LaunchConfiguration('occupancy_yaml')
    global_frame_id = LaunchConfiguration('global_frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_param_dir = LaunchConfiguration('localization_param_dir')

    # Pass-through to the core localization launch so dataset TF trees (Koide
    # bags) and the lidar frame can be selected from this single entry point.
    use_dataset_tf_tree = LaunchConfiguration('use_dataset_tf_tree')
    dataset_root_frame = LaunchConfiguration('dataset_root_frame')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')

    # Supervisor guards most likely to be tuned per scenario; the rest keep the
    # node's own defaults (see reinitialization_supervisor_policy).
    supervisor_min_candidate_score = LaunchConfiguration('supervisor_min_candidate_score')
    supervisor_max_attempts = LaunchConfiguration('supervisor_max_attempts')
    # G2's BBS query can take ~10-25 s; query/settle timeouts must allow for it.
    supervisor_query_timeout_sec = LaunchConfiguration('supervisor_query_timeout_sec')
    supervisor_settle_timeout_sec = LaunchConfiguration('supervisor_settle_timeout_sec')
    supervisor_max_walk_candidates = LaunchConfiguration('supervisor_max_walk_candidates')
    supervisor_enable_seed_motion = LaunchConfiguration('supervisor_enable_seed_motion_compensation')

    declared = [
        DeclareLaunchArgument(
            'cloud_topic', default_value='/velodyne_points',
            description='Scan topic shared by the localizer and the G2 search.'),
        DeclareLaunchArgument(
            'occupancy_yaml', default_value='',
            description='Occupancy grid YAML for the G2 BBS_2D search '
                        '(scripts/generate_occupancy_map_from_pcd.py).'),
        DeclareLaunchArgument(
            'global_frame_id', default_value='map',
            description='Map frame shared by all three nodes and /initialpose.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'localization_param_dir',
            default_value=os.path.join(pkg_share, 'param', 'localization.yaml'),
            description='Parameter file for the core lidar_localization node.'),
        DeclareLaunchArgument('use_dataset_tf_tree', default_value='false'),
        DeclareLaunchArgument('dataset_root_frame', default_value='camera_base'),
        DeclareLaunchArgument('lidar_frame_id', default_value='velodyne'),
        DeclareLaunchArgument(
            'supervisor_min_candidate_score', default_value='0.6',
            description='No reset is published below this candidate score.'),
        DeclareLaunchArgument(
            'supervisor_max_attempts', default_value='3',
            description='Hard ceiling on resets for one continuous problem.'),
        DeclareLaunchArgument(
            'supervisor_query_timeout_sec', default_value='10.0',
            description='Abandon a G2 query that returns nothing within this long '
                        '(raise above the BBS query latency, ~10-25 s).'),
        DeclareLaunchArgument(
            'supervisor_settle_timeout_sec', default_value='8.0',
            description='Time to observe post-reset recovery before counting the '
                        'attempt failed.'),
        DeclareLaunchArgument(
            'supervisor_max_walk_candidates', default_value='4',
            description='Walk at most this many candidates from one (possibly stale) '
                        'query before re-querying on a fresher scan; a stale query '
                        'can return a full list none of whose poses lock '
                        '(g3_live_closed_loop.md). Set high to walk the whole list.'),
        DeclareLaunchArgument(
            'supervisor_enable_seed_motion_compensation', default_value='false',
            description='Forward-extrapolate a candidate by the measured query->publish '
                        'latency so it lands where the moving vehicle is now, not where '
                        'it was when the (slow) BBS query was issued. Velocity is inferred '
                        'from successive query fixes (g3_live_closed_loop.md). Off by '
                        'default; helps only on a moving vehicle with slow queries.'),
        DeclareLaunchArgument(
            'g2_angular_resolution_deg', default_value='5.0',
            description='G2 BBS yaw sampling step; coarser = faster query, fresher '
                        'seed (e.g. 10.0).'),
        DeclareLaunchArgument(
            'g2_max_scan_points', default_value='512',
            description='G2 BBS scan points; fewer = faster query (e.g. 256).'),
        DeclareLaunchArgument(
            'g2_pyramid_depth', default_value='4',
            description='G2 BBS branch-and-bound pyramid depth.'),
        DeclareLaunchArgument(
            'g2_max_candidates', default_value='16',
            description='G2 ranked candidate count returned per query.'),
        DeclareLaunchArgument(
            'g2_use_cpp_backend', default_value='false',
            description='Use the compiled C++ BBS backend (bbs_cpp) when built; '
                        'falls back to the Python search if unavailable.'),
    ]

    # 1. Core localizer (also raises /reinitialization_requested + /alignment_status).
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'lidar_localization.launch.py')),
        launch_arguments={
            'localization_param_dir': localization_param_dir,
            'cloud_topic': cloud_topic,
            'use_sim_time': use_sim_time,
            'use_dataset_tf_tree': use_dataset_tf_tree,
            'dataset_root_frame': dataset_root_frame,
            'lidar_frame_id': lidar_frame_id,
        }.items())

    # 2. G2 on-demand global-localization service. The search-cost parameters are
    # exposed because query latency directly bounds live recovery: a candidate is
    # stale by the query duration on a moving vehicle (see g3_live_closed_loop.md),
    # so a coarser/faster setting trades a little accuracy for a fresher seed.
    global_localization = Node(
        package='lidar_localization_ros2',
        executable='global_localization_node.py',
        name='global_localization_node',
        output='screen',
        parameters=[{
            'occupancy_yaml': occupancy_yaml,
            'cloud_topic': cloud_topic,
            'global_frame_id': global_frame_id,
            'angular_resolution_deg': ParameterValue(
                LaunchConfiguration('g2_angular_resolution_deg'), value_type=float),
            'max_scan_points': ParameterValue(
                LaunchConfiguration('g2_max_scan_points'), value_type=int),
            'pyramid_depth': ParameterValue(
                LaunchConfiguration('g2_pyramid_depth'), value_type=int),
            'max_candidates': ParameterValue(
                LaunchConfiguration('g2_max_candidates'), value_type=int),
            'use_cpp_backend': ParameterValue(
                LaunchConfiguration('g2_use_cpp_backend'), value_type=bool),
            'use_sim_time': use_sim_time,
        }])

    # 3. G3 guarded automatic reinitialization supervisor (opt-in, guarded).
    supervisor = Node(
        package='lidar_localization_ros2',
        executable='reinitialization_supervisor_node.py',
        name='reinitialization_supervisor_node',
        output='screen',
        parameters=[{
            'query_service': '/global_localization_node/query',
            'alignment_status_topic': '/alignment_status',
            'initialpose_topic': '/initialpose',
            'global_frame_id': global_frame_id,
            'min_candidate_score': ParameterValue(
                supervisor_min_candidate_score, value_type=float),
            'max_attempts': ParameterValue(supervisor_max_attempts, value_type=int),
            'query_timeout_sec': ParameterValue(
                supervisor_query_timeout_sec, value_type=float),
            'settle_timeout_sec': ParameterValue(
                supervisor_settle_timeout_sec, value_type=float),
            'max_walk_candidates': ParameterValue(
                supervisor_max_walk_candidates, value_type=int),
            'enable_seed_motion_compensation': ParameterValue(
                supervisor_enable_seed_motion, value_type=bool),
            'use_sim_time': use_sim_time,
        }])

    return LaunchDescription(declared + [localization, global_localization, supervisor])
