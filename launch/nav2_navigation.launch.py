import os
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_optional_bool(value: str):
    normalized = value.strip().lower()
    if not normalized:
        return None
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"Invalid boolean launch argument value: {value}")


def _as_optional_float(value: str):
    normalized = value.strip()
    if not normalized:
        return None
    return float(normalized)


def _rewrite_nav2_params(
    nav2_params_file: str,
    pointcloud_topic: str,
    odom_topic: str,
    global_frame_id: str,
    odom_frame_id: str,
    base_frame_id: str,
    robot_radius,
) -> str:
    with Path(nav2_params_file).open("r", encoding="utf-8") as stream:
        params = yaml.safe_load(stream)

    local_costmap = params["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
    bt_navigator = params["bt_navigator"]["ros__parameters"]
    behavior_server = params["behavior_server"]["ros__parameters"]
    velocity_smoother = params["velocity_smoother"]["ros__parameters"]

    for layer_name in ("voxel_layer", "obstacle_layer"):
        local_layer = local_costmap.get(layer_name)
        if isinstance(local_layer, dict) and isinstance(local_layer.get("pointcloud"), dict):
            local_layer["pointcloud"]["topic"] = pointcloud_topic

        global_layer = global_costmap.get(layer_name)
        if isinstance(global_layer, dict) and isinstance(global_layer.get("pointcloud"), dict):
            global_layer["pointcloud"]["topic"] = pointcloud_topic

    bt_navigator["global_frame"] = global_frame_id
    bt_navigator["robot_base_frame"] = base_frame_id
    local_costmap["global_frame"] = odom_frame_id
    local_costmap["robot_base_frame"] = base_frame_id
    global_costmap["global_frame"] = global_frame_id
    global_costmap["robot_base_frame"] = base_frame_id
    behavior_server["global_frame"] = odom_frame_id
    behavior_server["robot_base_frame"] = base_frame_id
    bt_navigator["odom_topic"] = odom_topic
    velocity_smoother["odom_topic"] = odom_topic
    if robot_radius is not None:
        local_costmap["robot_radius"] = robot_radius
        global_costmap["robot_radius"] = robot_radius

    fd, rewritten_path = tempfile.mkstemp(prefix="lidar_localization_nav2_", suffix=".yaml")
    os.close(fd)
    with Path(rewritten_path).open("w", encoding="utf-8") as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
    return rewritten_path


def _rewrite_localization_params(
    localization_param_file: str,
    pcd_map_path: str,
    set_initial_pose,
    initial_pose_values: dict,
    enable_timer_publishing,
    pose_publish_frequency,
):
    should_rewrite = (
        bool(pcd_map_path)
        or set_initial_pose is not None
        or enable_timer_publishing is not None
        or pose_publish_frequency is not None
    )
    if not should_rewrite:
        return localization_param_file

    with Path(localization_param_file).open("r", encoding="utf-8") as stream:
        params = yaml.safe_load(stream)

    ros_params = params["/**"]["ros__parameters"]
    if pcd_map_path:
        ros_params["map_path"] = pcd_map_path
    if set_initial_pose is not None:
        ros_params["set_initial_pose"] = bool(set_initial_pose)
        if set_initial_pose:
            ros_params.update(initial_pose_values)
    if enable_timer_publishing is not None:
        ros_params["enable_timer_publishing"] = bool(enable_timer_publishing)
    if pose_publish_frequency is not None:
        ros_params["pose_publish_frequency"] = float(pose_publish_frequency)

    fd, rewritten_path = tempfile.mkstemp(prefix="lidar_localization_params_", suffix=".yaml")
    os.close(fd)
    with Path(rewritten_path).open("w", encoding="utf-8") as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
    return rewritten_path


def _resolve_effective_map_yaml(map_yaml: str, pointcloud_map_path: str, reference_csv: str, context) -> str:
    if map_yaml:
        map_yaml_path = Path(map_yaml).expanduser().resolve()
        if not map_yaml_path.exists():
            raise RuntimeError(f"map_yaml does not exist: {map_yaml_path}")
        with map_yaml_path.open("r", encoding="utf-8") as stream:
            map_config = yaml.safe_load(stream)
        image_path = Path(map_config["image"])
        if not image_path.is_absolute():
            image_path = map_yaml_path.parent / image_path
        if not image_path.exists():
            raise RuntimeError(f"map_yaml image does not exist: {image_path}")
        return str(map_yaml_path)

    generate_map = LaunchConfiguration("generate_map_from_pcd").perform(context).lower() in ("1", "true", "yes")
    if not generate_map:
        return ""

    if not pointcloud_map_path:
        raise RuntimeError("generate_map_from_pcd:=true requires pcd_map_path")
    if not reference_csv:
        raise RuntimeError("generate_map_from_pcd:=true requires reference_csv")

    output_dir = LaunchConfiguration("generated_map_output_dir").perform(context).strip()
    if output_dir:
        output_dir_path = Path(output_dir).expanduser().resolve()
        output_dir_path.mkdir(parents=True, exist_ok=True)
    else:
        output_dir_path = Path(tempfile.mkdtemp(prefix="lidar_localization_nav2_map_"))

    map_name = LaunchConfiguration("generated_map_name").perform(context).strip() or "nav2_generated_map"
    route_padding_m = LaunchConfiguration("generated_map_route_padding_m").perform(context)
    resolution = LaunchConfiguration("generated_map_resolution").perform(context)
    ground_band_m = LaunchConfiguration("generated_map_ground_band_m").perform(context)
    package_prefix = Path(get_package_prefix("lidar_localization_ros2"))
    generator = package_prefix / "lib" / "lidar_localization_ros2" / "generate_occupancy_map_from_pcd.py"

    command = [
        sys.executable,
        str(generator),
        "--pcd",
        str(Path(pointcloud_map_path).expanduser().resolve()),
        "--reference-csv",
        str(Path(reference_csv).expanduser().resolve()),
        "--route-padding-m",
        route_padding_m,
        "--ground-band-m",
        ground_band_m,
        "--resolution",
        resolution,
        "--output-dir",
        str(output_dir_path),
        "--map-name",
        map_name,
    ]
    subprocess.run(command, check=True)
    return str((output_dir_path / f"{map_name}.yaml").resolve())


def _start_localization(context, *args, **kwargs):
    if _as_optional_bool(LaunchConfiguration("use_odom_localization_demo").perform(context)):
        global_frame_id = LaunchConfiguration("global_frame_id").perform(context)
        odom_frame_id = LaunchConfiguration("odom_frame_id").perform(context)
        base_frame_id = LaunchConfiguration("base_frame_id").perform(context)
        odom_topic = LaunchConfiguration("odom_topic").perform(context)
        initial_pose_values = {
            "initial_pose_x": float(LaunchConfiguration("initial_pose_x").perform(context)),
            "initial_pose_y": float(LaunchConfiguration("initial_pose_y").perform(context)),
            "initial_pose_z": float(LaunchConfiguration("initial_pose_z").perform(context)),
            "initial_pose_qx": float(LaunchConfiguration("initial_pose_qx").perform(context)),
            "initial_pose_qy": float(LaunchConfiguration("initial_pose_qy").perform(context)),
            "initial_pose_qz": float(LaunchConfiguration("initial_pose_qz").perform(context)),
            "initial_pose_qw": float(LaunchConfiguration("initial_pose_qw").perform(context)),
        }
        return [
            LogInfo(
                msg=(
                    "Using odom localization demo backend with "
                    f"{global_frame_id}->{odom_frame_id} and {odom_topic}"
                )
            ),
            Node(
                package="lidar_localization_ros2",
                executable="publish_pose_from_odom.py",
                name="pose_from_odom_publisher",
                output="screen",
                parameters=[
                    {
                        "odom_topic": odom_topic,
                        "pose_topic": "/localization/pose_with_covariance",
                        "global_frame_id": global_frame_id,
                        "odom_frame_id": odom_frame_id,
                        "base_frame_id": base_frame_id,
                        **initial_pose_values,
                    }
                ],
            ),
        ]

    package_share = get_package_share_directory("lidar_localization_ros2")
    localization_launch = os.path.join(package_share, "launch", "nav2_lidar_localization.launch.py")

    localization_param_dir = LaunchConfiguration("localization_param_dir").perform(context)
    cloud_topic = LaunchConfiguration("cloud_topic").perform(context)
    twist_topic = LaunchConfiguration("twist_topic").perform(context)
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    global_frame_id = LaunchConfiguration("global_frame_id").perform(context)
    odom_frame_id = LaunchConfiguration("odom_frame_id").perform(context)
    base_frame_id = LaunchConfiguration("base_frame_id").perform(context)
    publish_lidar_tf = LaunchConfiguration("publish_lidar_tf").perform(context)
    lidar_frame_id = LaunchConfiguration("lidar_frame_id").perform(context)
    publish_imu_tf = LaunchConfiguration("publish_imu_tf").perform(context)
    imu_frame_id = LaunchConfiguration("imu_frame_id").perform(context)
    pcd_map_path = LaunchConfiguration("pcd_map_path").perform(context).strip()
    set_initial_pose = _as_optional_bool(LaunchConfiguration("set_initial_pose").perform(context))
    enable_timer_publishing = _as_optional_bool(
        LaunchConfiguration("localizer_enable_timer_publishing").perform(context)
    )
    pose_publish_frequency = _as_optional_float(
        LaunchConfiguration("localizer_pose_publish_frequency").perform(context)
    )
    initial_pose_values = {
        "initial_pose_x": float(LaunchConfiguration("initial_pose_x").perform(context)),
        "initial_pose_y": float(LaunchConfiguration("initial_pose_y").perform(context)),
        "initial_pose_z": float(LaunchConfiguration("initial_pose_z").perform(context)),
        "initial_pose_qx": float(LaunchConfiguration("initial_pose_qx").perform(context)),
        "initial_pose_qy": float(LaunchConfiguration("initial_pose_qy").perform(context)),
        "initial_pose_qz": float(LaunchConfiguration("initial_pose_qz").perform(context)),
        "initial_pose_qw": float(LaunchConfiguration("initial_pose_qw").perform(context)),
    }

    effective_localization_params = _rewrite_localization_params(
        localization_param_file=localization_param_dir,
        pcd_map_path=pcd_map_path,
        set_initial_pose=set_initial_pose,
        initial_pose_values=initial_pose_values,
        enable_timer_publishing=enable_timer_publishing,
        pose_publish_frequency=pose_publish_frequency,
    )

    messages = []
    if pcd_map_path:
        messages.append(LogInfo(msg=f"Using pointcloud map override: {pcd_map_path}"))
    if set_initial_pose is True:
        messages.append(
            LogInfo(
                msg=(
                    "Using initial pose override: "
                    f"({initial_pose_values['initial_pose_x']}, {initial_pose_values['initial_pose_y']}, "
                    f"{initial_pose_values['initial_pose_z']})"
                )
            )
        )

    messages.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "localization_param_dir": effective_localization_params,
                "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
                "enable_map_odom_tf": LaunchConfiguration("localizer_enable_map_odom_tf").perform(context),
                "global_frame_id": global_frame_id,
                "odom_frame_id": odom_frame_id,
                "base_frame_id": base_frame_id,
                "cloud_topic": cloud_topic,
                "twist_topic": twist_topic,
                "imu_topic": imu_topic,
                "publish_lidar_tf": publish_lidar_tf,
                "lidar_frame_id": lidar_frame_id,
                "lidar_tf_x": LaunchConfiguration("lidar_tf_x").perform(context),
                "lidar_tf_y": LaunchConfiguration("lidar_tf_y").perform(context),
                "lidar_tf_z": LaunchConfiguration("lidar_tf_z").perform(context),
                "lidar_tf_roll": LaunchConfiguration("lidar_tf_roll").perform(context),
                "lidar_tf_pitch": LaunchConfiguration("lidar_tf_pitch").perform(context),
                "lidar_tf_yaw": LaunchConfiguration("lidar_tf_yaw").perform(context),
                "publish_imu_tf": publish_imu_tf,
                "imu_frame_id": imu_frame_id,
                "imu_tf_x": LaunchConfiguration("imu_tf_x").perform(context),
                "imu_tf_y": LaunchConfiguration("imu_tf_y").perform(context),
                "imu_tf_z": LaunchConfiguration("imu_tf_z").perform(context),
                "imu_tf_roll": LaunchConfiguration("imu_tf_roll").perform(context),
                "imu_tf_pitch": LaunchConfiguration("imu_tf_pitch").perform(context),
                "imu_tf_yaw": LaunchConfiguration("imu_tf_yaw").perform(context),
            }.items(),
        )
    )
    return messages


def _maybe_nav2(context, *args, **kwargs):
    launch_nav2 = LaunchConfiguration("launch_nav2").perform(context).lower() in ("1", "true", "yes")
    if not launch_nav2:
        return [LogInfo(msg="launch_nav2:=false, starting lidar_localization only")]

    pcd_map_path = LaunchConfiguration("pcd_map_path").perform(context).strip()
    reference_csv = LaunchConfiguration("reference_csv").perform(context).strip()
    try:
        map_yaml = _resolve_effective_map_yaml(
            map_yaml=LaunchConfiguration("map_yaml").perform(context).strip(),
            pointcloud_map_path=pcd_map_path,
            reference_csv=reference_csv,
            context=context,
        )
    except Exception as exc:
        return [LogInfo(msg=f"Skipping Nav2 stack: {exc}")]
    if not map_yaml:
        return [LogInfo(msg="map_yaml is empty, skipping Nav2 stack and starting lidar_localization only")]

    try:
        nav2_bringup_share = get_package_share_directory("nav2_bringup")
    except Exception:
        return [LogInfo(msg="nav2_bringup not found, skipping Nav2 stack")]

    required_packages = ["nav2_map_server", "nav2_lifecycle_manager"]
    for package_name in required_packages:
        try:
            get_package_share_directory(package_name)
        except Exception:
            return [LogInfo(msg=f"{package_name} not found, skipping Nav2 stack")]

    nav2_params_file = LaunchConfiguration("nav2_params_file").perform(context)
    nav2_params_path = Path(nav2_params_file).expanduser().resolve()
    if not nav2_params_path.exists():
        return [LogInfo(msg=f"nav2_params_file does not exist, skipping Nav2 stack: {nav2_params_path}")]
    pointcloud_topic = LaunchConfiguration("pointcloud_topic").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    global_frame_id = LaunchConfiguration("global_frame_id").perform(context)
    odom_frame_id = LaunchConfiguration("odom_frame_id").perform(context)
    base_frame_id = LaunchConfiguration("base_frame_id").perform(context)
    robot_radius = _as_optional_float(LaunchConfiguration("robot_radius").perform(context))
    use_sim_time_arg = LaunchConfiguration("use_sim_time").perform(context)
    autostart_arg = LaunchConfiguration("autostart").perform(context)
    use_sim_time = _as_optional_bool(use_sim_time_arg)
    autostart = _as_optional_bool(autostart_arg)
    if use_sim_time is None:
        use_sim_time = False
        use_sim_time_arg = "false"
    if autostart is None:
        autostart = True
        autostart_arg = "true"
    log_level = LaunchConfiguration("log_level").perform(context)

    rewritten_nav2_params = _rewrite_nav2_params(
        str(nav2_params_path),
        pointcloud_topic,
        odom_topic,
        global_frame_id,
        odom_frame_id,
        base_frame_id,
        robot_radius,
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    return [
        LogInfo(
            msg=(
                "Launching Nav2 stack with "
                f"map_yaml={map_yaml}, pointcloud_topic={pointcloud_topic}, odom_topic={odom_topic}"
            )
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[rewritten_nav2_params, {"yaml_filename": map_yaml}, {"use_sim_time": use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": ["map_server"]},
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time_arg,
                "params_file": rewritten_nav2_params,
                "autostart": autostart_arg,
                "log_level": log_level,
            }.items(),
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("lidar_localization_ros2")
    localization_launch = os.path.join(package_share, "launch", "nav2_lidar_localization.launch.py")
    default_localization_params = os.path.join(package_share, "param", "nav2_ndt_urban.yaml")
    default_nav2_params = os.path.join(package_share, "param", "nav2_humble_pointcloud.yaml")

    localization_param_dir = LaunchConfiguration("localization_param_dir")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    cloud_topic = LaunchConfiguration("cloud_topic")
    twist_topic = LaunchConfiguration("twist_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    global_frame_id = LaunchConfiguration("global_frame_id")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")
    pcd_map_path = LaunchConfiguration("pcd_map_path")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    map_yaml = LaunchConfiguration("map_yaml")
    reference_csv = LaunchConfiguration("reference_csv")
    odom_topic = LaunchConfiguration("odom_topic")
    robot_radius = LaunchConfiguration("robot_radius")
    publish_localizer_pose_odom = LaunchConfiguration("publish_localizer_pose_odom")
    publish_identity_odom = LaunchConfiguration("publish_identity_odom")
    identity_odom_rate_hz = LaunchConfiguration("identity_odom_rate_hz")
    publish_twist_odom = LaunchConfiguration("publish_twist_odom")
    twist_odom_max_dt_sec = LaunchConfiguration("twist_odom_max_dt_sec")
    publish_cmd_vel_odom = LaunchConfiguration("publish_cmd_vel_odom")
    cmd_vel_odom_rate_hz = LaunchConfiguration("cmd_vel_odom_rate_hz")
    enable_reinitialization_supervisor = LaunchConfiguration("enable_reinitialization_supervisor")
    reinitialization_supervisor_use_latest_pose = LaunchConfiguration(
        "reinitialization_supervisor_use_latest_pose"
    )
    reinitialization_supervisor_publish_count = LaunchConfiguration(
        "reinitialization_supervisor_publish_count"
    )
    reinitialization_supervisor_publish_interval_sec = LaunchConfiguration(
        "reinitialization_supervisor_publish_interval_sec"
    )
    reinitialization_supervisor_cooldown_sec = LaunchConfiguration(
        "reinitialization_supervisor_cooldown_sec"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_param_dir", default_value=default_localization_params),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("global_frame_id", default_value="map"),
            DeclareLaunchArgument("odom_frame_id", default_value="odom"),
            DeclareLaunchArgument("base_frame_id", default_value="base_link"),
            DeclareLaunchArgument("localizer_enable_map_odom_tf", default_value="true"),
            DeclareLaunchArgument("cloud_topic", default_value="/velodyne_points"),
            DeclareLaunchArgument("twist_topic", default_value="/twist"),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("publish_lidar_tf", default_value="true"),
            DeclareLaunchArgument("lidar_frame_id", default_value="velodyne"),
            DeclareLaunchArgument("lidar_tf_x", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_z", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("publish_imu_tf", default_value="true"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("imu_tf_x", default_value="0.0"),
            DeclareLaunchArgument("imu_tf_y", default_value="0.0"),
            DeclareLaunchArgument("imu_tf_z", default_value="0.0"),
            DeclareLaunchArgument("imu_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("imu_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("imu_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("localizer_enable_timer_publishing", default_value="true"),
            DeclareLaunchArgument("localizer_pose_publish_frequency", default_value="10.0"),
            DeclareLaunchArgument("use_odom_localization_demo", default_value="false"),
            DeclareLaunchArgument("pcd_map_path", default_value=""),
            DeclareLaunchArgument("pointcloud_topic", default_value="/velodyne_points"),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument("reference_csv", default_value=""),
            DeclareLaunchArgument("generate_map_from_pcd", default_value="false"),
            DeclareLaunchArgument("generated_map_output_dir", default_value=""),
            DeclareLaunchArgument("generated_map_name", default_value="nav2_generated_map"),
            DeclareLaunchArgument("generated_map_route_padding_m", default_value="20.0"),
            DeclareLaunchArgument("generated_map_resolution", default_value="0.25"),
            DeclareLaunchArgument("generated_map_ground_band_m", default_value="1.0"),
            DeclareLaunchArgument("set_initial_pose", default_value=""),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_z", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_qx", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_qy", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_qz", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_qw", default_value="1.0"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("robot_radius", default_value=""),
            DeclareLaunchArgument("publish_localizer_pose_odom", default_value="false"),
            DeclareLaunchArgument("publish_identity_odom", default_value="false"),
            DeclareLaunchArgument("identity_odom_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("publish_twist_odom", default_value="false"),
            DeclareLaunchArgument("twist_odom_max_dt_sec", default_value="0.5"),
            DeclareLaunchArgument("publish_cmd_vel_odom", default_value="false"),
            DeclareLaunchArgument("cmd_vel_odom_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("enable_reinitialization_supervisor", default_value="false"),
            DeclareLaunchArgument("reinitialization_supervisor_use_latest_pose", default_value="false"),
            DeclareLaunchArgument("reinitialization_supervisor_publish_count", default_value="1"),
            DeclareLaunchArgument("reinitialization_supervisor_publish_interval_sec", default_value="0.2"),
            DeclareLaunchArgument("reinitialization_supervisor_cooldown_sec", default_value="15.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("launch_nav2", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="lidar_localization_ros2",
                executable="publish_odom_from_localization.py",
                name="odom_from_localization_publisher",
                output="screen",
                condition=IfCondition(publish_localizer_pose_odom),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "pose_topic": "/localization/pose_with_covariance",
                        "odom_topic": odom_topic,
                        "odom_frame_id": odom_frame_id,
                        "base_frame_id": base_frame_id,
                        "publish_tf": True,
                    }
                ],
            ),
            Node(
                package="lidar_localization_ros2",
                executable="publish_identity_odom.py",
                name="identity_odom_publisher",
                output="screen",
                condition=IfCondition(publish_identity_odom),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "odom_topic": odom_topic,
                        "odom_frame_id": odom_frame_id,
                        "base_frame_id": base_frame_id,
                        "rate_hz": identity_odom_rate_hz,
                        "publish_tf": True,
                    }
                ],
            ),
            Node(
                package="lidar_localization_ros2",
                executable="publish_odom_from_twist.py",
                name="twist_odom_publisher",
                output="screen",
                condition=IfCondition(publish_twist_odom),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "twist_topic": twist_topic,
                        "odom_topic": odom_topic,
                        "odom_frame_id": odom_frame_id,
                        "base_frame_id": base_frame_id,
                        "max_dt_sec": twist_odom_max_dt_sec,
                        "publish_tf": True,
                    }
                ],
            ),
            Node(
                package="lidar_localization_ros2",
                executable="publish_cmd_vel_odom.py",
                name="cmd_vel_odom_publisher",
                output="screen",
                condition=IfCondition(publish_cmd_vel_odom),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "cmd_vel_topic": "/cmd_vel",
                        "odom_topic": odom_topic,
                        "odom_frame_id": odom_frame_id,
                        "base_frame_id": base_frame_id,
                        "rate_hz": cmd_vel_odom_rate_hz,
                        "publish_tf": True,
                    }
                ],
            ),
            Node(
                package="lidar_localization_ros2",
                executable="republish_initialpose_on_reinit.py",
                name="reinitialization_supervisor",
                output="screen",
                condition=IfCondition(enable_reinitialization_supervisor),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "request_topic": "/reinitialization_requested",
                        "initialpose_topic": "/initialpose",
                        "pose_topic": "/localization/pose_with_covariance",
                        "global_frame_id": global_frame_id,
                        "use_latest_pose": reinitialization_supervisor_use_latest_pose,
                        "publish_count": reinitialization_supervisor_publish_count,
                        "publish_interval_sec": reinitialization_supervisor_publish_interval_sec,
                        "republish_cooldown_sec": reinitialization_supervisor_cooldown_sec,
                        "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose_z": LaunchConfiguration("initial_pose_z"),
                        "initial_pose_qx": LaunchConfiguration("initial_pose_qx"),
                        "initial_pose_qy": LaunchConfiguration("initial_pose_qy"),
                        "initial_pose_qz": LaunchConfiguration("initial_pose_qz"),
                        "initial_pose_qw": LaunchConfiguration("initial_pose_qw"),
                    }
                ],
            ),
            OpaqueFunction(function=_start_localization),
            OpaqueFunction(function=_maybe_nav2),
        ]
    )
