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

Example (Koide outdoor_hard_01a, Livox frame)::

    ros2 launch lidar_localization_ros2 global_localization_recovery.launch.py \\
        cloud_topic:=/livox/points imu_topic:=/livox/imu \\
        occupancy_yaml:=/path/to/occupancy.yaml \\
        localization_param_dir:=/path/to/localization.yaml \\
        base_frame_id:=livox_frame lidar_frame_id:=livox_frame \\
        publish_lidar_tf:=false use_imu_preintegration:=true \\
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
    map_path = LaunchConfiguration('map_path')
    global_frame_id = LaunchConfiguration('global_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_param_dir = LaunchConfiguration('localization_param_dir')

    # Pass-through to the core localization launch so dataset TF trees (Koide
    # bags) and the lidar frame can be selected from this single entry point.
    twist_topic = LaunchConfiguration('twist_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    use_dataset_tf_tree = LaunchConfiguration('use_dataset_tf_tree')
    dataset_root_frame = LaunchConfiguration('dataset_root_frame')
    publish_lidar_tf = LaunchConfiguration('publish_lidar_tf')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    use_imu_preintegration = LaunchConfiguration('use_imu_preintegration')
    imu_preintegration_use_base_frame_transform = LaunchConfiguration(
        'imu_preintegration_use_base_frame_transform')
    use_continuous_time_deskew = LaunchConfiguration('use_continuous_time_deskew')
    continuous_time_deskew_reference_time_sec = LaunchConfiguration(
        'continuous_time_deskew_reference_time_sec')

    # Supervisor guards most likely to be tuned per scenario; the rest keep the
    # node's own defaults (see reinitialization_supervisor_policy).
    supervisor_min_candidate_score = LaunchConfiguration('supervisor_min_candidate_score')
    supervisor_max_attempts = LaunchConfiguration('supervisor_max_attempts')
    # G2's BBS query can take ~10-25 s; query/settle timeouts must allow for it.
    supervisor_query_timeout_sec = LaunchConfiguration('supervisor_query_timeout_sec')
    supervisor_settle_timeout_sec = LaunchConfiguration('supervisor_settle_timeout_sec')
    supervisor_recovery_confirmation_samples = LaunchConfiguration(
        'supervisor_recovery_confirmation_samples')
    supervisor_max_walk_candidates = LaunchConfiguration('supervisor_max_walk_candidates')
    supervisor_confirm_cross_check = LaunchConfiguration('supervisor_confirm_cross_check')
    supervisor_cross_check_mismatch_m = LaunchConfiguration(
        'supervisor_cross_check_mismatch_m')
    supervisor_enable_seed_motion = LaunchConfiguration('supervisor_enable_seed_motion_compensation')
    supervisor_max_seed_speed_mps = LaunchConfiguration('supervisor_max_seed_speed_mps')
    supervisor_max_seed_latency_sec = LaunchConfiguration('supervisor_max_seed_latency_sec')
    supervisor_seed_velocity_max_age_sec = LaunchConfiguration(
        'supervisor_seed_velocity_max_age_sec')
    supervisor_seed_motion_wall_fallback = LaunchConfiguration(
        'supervisor_seed_motion_wall_fallback')
    supervisor_event_log_csv = LaunchConfiguration('supervisor_event_log_csv')
    supervisor_reset_default_z_m = LaunchConfiguration('supervisor_reset_default_z_m')
    supervisor_prefer_reset_default_z_m = LaunchConfiguration(
        'supervisor_prefer_reset_default_z_m')
    supervisor_seed_motion_skip_registration_fitness_threshold = LaunchConfiguration(
        'supervisor_seed_motion_skip_registration_fitness_threshold')
    g2_ndt_scan_voxel_leaf_size = LaunchConfiguration('g2_ndt_scan_voxel_leaf_size')
    g2_ndt_target_voxel_leaf_size = LaunchConfiguration('g2_ndt_target_voxel_leaf_size')

    declared = [
        DeclareLaunchArgument(
            'cloud_topic', default_value='/velodyne_points',
            description='Scan topic shared by the localizer and the G2 search.'),
        DeclareLaunchArgument(
            'occupancy_yaml', default_value='',
            description='Occupancy grid YAML for the G2 BBS_2D search '
                        '(scripts/generate_occupancy_map_from_pcd.py).'),
        DeclareLaunchArgument(
            'map_path', default_value='',
            description='3D map PCD/PLY for optional G2 NDT registration scoring.'),
        DeclareLaunchArgument(
            'g2_enable_registration_scoring', default_value='true',
            description='Re-rank BBS candidates by NDT fitness when map_path is set.'),
        DeclareLaunchArgument(
            'g2_registration_score_gate', default_value='6.0',
            description='NDT fitness gate used to map registration scores to [0, 1].'),
        DeclareLaunchArgument(
            'g2_ndt_num_threads', default_value='1',
            description='Threads for G2 NDT registration scoring. KEEP AT 1: pclomp '
                        'NDT_OMP with the default KDTREE neighborhood search is not '
                        'thread-safe -- 4 threads degraded HDL registration fitness '
                        'from 0.03-0.9 to 5-12 (all fixes rejected) with no runtime '
                        'gain. Cut query runtime with g2_max_candidates instead; '
                        'revisit after wiring setNeighborhoodSearchMethod(DIRECT7).'),
        DeclareLaunchArgument(
            'g2_registration_refine_candidates', default_value='false',
            description='Report the NDT-refined pose of converged candidates instead '
                        'of the raw BBS cell pose. Off by default: on high-aliasing '
                        'maps refinement snaps wrong hypotheses to locally-perfect '
                        'poses and collapses walk-candidate diversity.'),
        DeclareLaunchArgument(
            'g2_registration_seed_z_m', default_value='0.0',
            description='Map-frame z used for NDT registration scoring on 2D candidates.'),
        DeclareLaunchArgument(
            'global_frame_id', default_value='map',
            description='Map frame shared by all three nodes and /initialpose.'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'localization_param_dir',
            default_value=os.path.join(pkg_share, 'param', 'localization.yaml'),
            description='Parameter file for the core lidar_localization node.'),
        DeclareLaunchArgument('twist_topic', default_value='/twist'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('use_dataset_tf_tree', default_value='false'),
        DeclareLaunchArgument('dataset_root_frame', default_value='camera_base'),
        DeclareLaunchArgument('publish_lidar_tf', default_value='true'),
        DeclareLaunchArgument('lidar_frame_id', default_value='velodyne'),
        DeclareLaunchArgument('publish_imu_tf', default_value='false'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('use_imu_preintegration', default_value='true'),
        DeclareLaunchArgument(
            'imu_preintegration_use_base_frame_transform', default_value='false'),
        DeclareLaunchArgument('use_continuous_time_deskew', default_value='false'),
        DeclareLaunchArgument(
            'continuous_time_deskew_reference_time_sec', default_value='0.0'),
        DeclareLaunchArgument(
            'supervisor_min_candidate_score', default_value='0.15',
            description='No reset is published below this candidate score. '
                        'Use ~0.15 when G2 registration scoring maps NDT fitness to '
                        '[0, 1]; use ~0.6 for BBS-only scores.'),
        DeclareLaunchArgument(
            'supervisor_max_attempts', default_value='5',
            description='Hard ceiling on query attempts for one continuous problem.'),
        DeclareLaunchArgument(
            'supervisor_query_timeout_sec', default_value='45.0',
            description='Abandon a G2 query that returns nothing within this long '
                        '(raise above the BBS+NDT query latency, ~15-30 s).'),
        DeclareLaunchArgument(
            'supervisor_settle_timeout_sec', default_value='20.0',
            description='Time to observe post-reset recovery before counting the '
                        'attempt failed (wall seconds; allow headroom for slow bag replay).'),
        DeclareLaunchArgument(
            'supervisor_recovery_confirmation_samples', default_value='3',
            description='Consecutive post-reset low-fitness observations required '
                        'before recovery is confirmed.'),
        DeclareLaunchArgument(
            'supervisor_recovery_fitness_threshold', default_value='1.5',
            description='Post-reset fitness below this confirms recovery. Some map '
                        'sections score ~1.5-1.7 even when tracking is correct; raise '
                        'with care (a looser threshold also admits false confirms).'),
        DeclareLaunchArgument(
            'supervisor_max_walk_candidates', default_value='4',
            description='Walk at most this many candidates from one (possibly stale) '
                        'query before re-querying on a fresher scan; a stale query '
                        'can return a full list none of whose poses lock '
                        '(g3_live_closed_loop.md). Set high to walk the whole list.'),
        DeclareLaunchArgument(
            'supervisor_confirm_cross_check', default_value='true',
            description='After post-reset recovery evidence, issue one more G2 query '
                        'and compare the fresh fix to the localizer pose to reject '
                        'along-corridor alias locks that pass fitness alone.'),
        DeclareLaunchArgument(
            'supervisor_cross_check_mismatch_m', default_value='5.0',
            description='Euclidean xy distance (m) above which the verify query is '
                        'treated as an alias and recovery is rejected.'),
        DeclareLaunchArgument(
            'supervisor_enable_seed_motion_compensation', default_value='false',
            description='Forward-extrapolate a candidate by the measured query->publish '
                        'latency so it lands where the moving vehicle is now, not where '
                        'it was when the (slow) BBS query was issued. Velocity is inferred '
                        'from successive query fixes (g3_live_closed_loop.md). Off by '
                        'default; helps only on a moving vehicle with slow queries.'),
        DeclareLaunchArgument(
            'supervisor_max_seed_speed_mps', default_value='30.0',
            description='Reject seed-motion velocity estimates faster than this; lower '
                        'for slow bags to avoid compensating between aliased wrong '
                        'candidates.'),
        DeclareLaunchArgument(
            'supervisor_max_seed_latency_sec', default_value='30.0',
            description='Clamp seed-motion compensation latency to this many seconds.'),
        DeclareLaunchArgument(
            'supervisor_seed_velocity_max_age_sec', default_value='60.0',
            description='Reject pose-derived seed velocity older than this many seconds; '
                        'constant-velocity extrapolation decays under cornering.'),
        DeclareLaunchArgument(
            'supervisor_seed_motion_wall_fallback', default_value='false',
            description='When true, fall back to wall-clock seed motion compensation '
                        'if bag-clock sim fix-to-fix velocity is unavailable. Off by '
                        'default; wall-clock paths have produced garbage seeds.'),
        DeclareLaunchArgument(
            'supervisor_event_log_csv', default_value='',
            description='Optional CSV path for supervisor recovery events.'),
        DeclareLaunchArgument(
            'supervisor_reset_default_z_m', default_value='0.0',
            description='Fallback map-frame z for /initialpose seeds.'),
        DeclareLaunchArgument(
            'supervisor_prefer_reset_default_z_m', default_value='false',
            description='When true, always use reset_default_z_m for seed z instead of '
                        'the last /pcl_pose z (outdoor kidnapped-start helper).'),
        DeclareLaunchArgument(
            'supervisor_seed_motion_skip_registration_fitness_threshold',
            default_value='1.0',
            description='Skip seed motion compensation when a candidate registration '
                        'fitness is at or below this value.'),
        DeclareLaunchArgument(
            'g2_ndt_scan_voxel_leaf_size', default_value='1.0',
            description='G2 NDT scoring scan voxel size; 0 disables downsampling.'),
        DeclareLaunchArgument(
            'g2_ndt_target_voxel_leaf_size', default_value='0.2',
            description='G2 NDT scoring map voxel size (match localizer target).'),
        DeclareLaunchArgument(
            'g2_angular_resolution_deg', default_value='10.0',
            description='G2 BBS yaw sampling step; coarser = faster query, fresher '
                        'seed (e.g. 10.0).'),
        DeclareLaunchArgument(
            'g2_max_scan_points', default_value='256',
            description='G2 BBS scan points; fewer = faster query (e.g. 256).'),
        DeclareLaunchArgument(
            'g2_pyramid_depth', default_value='4',
            description='G2 BBS branch-and-bound pyramid depth.'),
        DeclareLaunchArgument(
            'g2_max_candidates', default_value='16',
            description='G2 ranked candidate count returned per query.'),
        DeclareLaunchArgument(
            'g2_nms_radius_m', default_value='2.0',
            description='G2 candidate non-maximum suppression radius. Lower values '
                        'keep more near-by/yaw-alternative hypotheses for candidate '
                        'walking; higher values improve spatial diversity.'),
        DeclareLaunchArgument(
            'g2_use_cpp_backend', default_value='true',
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
            'twist_topic': twist_topic,
            'imu_topic': imu_topic,
            'global_frame_id': global_frame_id,
            'odom_frame_id': odom_frame_id,
            'base_frame_id': base_frame_id,
            'use_sim_time': use_sim_time,
            'use_dataset_tf_tree': use_dataset_tf_tree,
            'dataset_root_frame': dataset_root_frame,
            'publish_lidar_tf': publish_lidar_tf,
            'lidar_frame_id': lidar_frame_id,
            'publish_imu_tf': publish_imu_tf,
            'imu_frame_id': imu_frame_id,
            'use_imu_preintegration': use_imu_preintegration,
            'imu_preintegration_use_base_frame_transform':
                imu_preintegration_use_base_frame_transform,
            'use_continuous_time_deskew': use_continuous_time_deskew,
            'continuous_time_deskew_reference_time_sec':
                continuous_time_deskew_reference_time_sec,
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
            'map_path': map_path,
            'enable_registration_scoring': ParameterValue(
                LaunchConfiguration('g2_enable_registration_scoring'), value_type=bool),
            'registration_score_gate': ParameterValue(
                LaunchConfiguration('g2_registration_score_gate'), value_type=float),
            'ndt_num_threads': ParameterValue(
                LaunchConfiguration('g2_ndt_num_threads'), value_type=int),
            'registration_refine_candidates': ParameterValue(
                LaunchConfiguration('g2_registration_refine_candidates'),
                value_type=bool),
            'registration_seed_z_m': ParameterValue(
                LaunchConfiguration('g2_registration_seed_z_m'), value_type=float),
            'ndt_scan_voxel_leaf_size': ParameterValue(
                g2_ndt_scan_voxel_leaf_size, value_type=float),
            'ndt_target_voxel_leaf_size': ParameterValue(
                g2_ndt_target_voxel_leaf_size, value_type=float),
            'angular_resolution_deg': ParameterValue(
                LaunchConfiguration('g2_angular_resolution_deg'), value_type=float),
            'max_scan_points': ParameterValue(
                LaunchConfiguration('g2_max_scan_points'), value_type=int),
            'pyramid_depth': ParameterValue(
                LaunchConfiguration('g2_pyramid_depth'), value_type=int),
            'max_candidates': ParameterValue(
                LaunchConfiguration('g2_max_candidates'), value_type=int),
            'nms_radius_m': ParameterValue(
                LaunchConfiguration('g2_nms_radius_m'), value_type=float),
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
            'recovery_confirmation_samples': ParameterValue(
                supervisor_recovery_confirmation_samples, value_type=int),
            'recovery_fitness_threshold': ParameterValue(
                LaunchConfiguration('supervisor_recovery_fitness_threshold'),
                value_type=float),
            'max_walk_candidates': ParameterValue(
                supervisor_max_walk_candidates, value_type=int),
            'confirm_cross_check': ParameterValue(
                supervisor_confirm_cross_check, value_type=bool),
            'cross_check_mismatch_m': ParameterValue(
                supervisor_cross_check_mismatch_m, value_type=float),
            'enable_seed_motion_compensation': ParameterValue(
                supervisor_enable_seed_motion, value_type=bool),
            'max_seed_speed_mps': ParameterValue(
                supervisor_max_seed_speed_mps, value_type=float),
            'max_seed_latency_sec': ParameterValue(
                supervisor_max_seed_latency_sec, value_type=float),
            'seed_velocity_max_age_sec': ParameterValue(
                supervisor_seed_velocity_max_age_sec, value_type=float),
            'seed_motion_wall_fallback': ParameterValue(
                supervisor_seed_motion_wall_fallback, value_type=bool),
            'reset_default_z_m': ParameterValue(
                supervisor_reset_default_z_m, value_type=float),
            'prefer_reset_default_z_m': ParameterValue(
                supervisor_prefer_reset_default_z_m, value_type=bool),
            'seed_motion_skip_registration_fitness_threshold': ParameterValue(
                supervisor_seed_motion_skip_registration_fitness_threshold,
                value_type=float),
            'event_log_csv': supervisor_event_log_csv,
            'use_sim_time': use_sim_time,
        }])

    return LaunchDescription(declared + [localization, global_localization, supervisor])
