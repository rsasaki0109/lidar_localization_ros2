# Mapless Public Dataset Workflow

## Goal

Use a public LiDAR + IMU dataset that has no packaged pointcloud map, while keeping map creation and localization evaluation separated.

## Rule

Do not build the map and evaluate localization on the same run if you want a meaningful result.

Use:

1. `mapping run`
2. `localization run`
3. optional `Nav2 occupancy-map generation`

## Bundle Scaffolding

Start from the example spec:

- `param/benchmark/mapless_public_dataset_bundle.example.yaml`
- `param/benchmark/boreas_mapless_public_dataset_bundle.example.yaml` for the Boreas starter path

Create a real copy, replace the placeholder paths, then scaffold the bundle:

```bash
source scripts/setup_local_env.sh
ros2 run lidar_localization_ros2 scaffold_mapless_public_dataset_bundle.py \
  --spec /absolute/path/to/mapless_public_dataset_bundle.yaml
```

The bundle contains:

- `run_mapping.sh`
- `run_extract_reference.sh` when the spec includes a `reference:` section
- `localization_run_manifest.yaml`
- `run_localization_benchmark.sh`
- `run_generate_nav2_map.sh`
- `README.md`

Path rule for bundle outputs:

- `bundle.generated_map_path`, `bundle.localization_output_dir`, and `bundle.nav2_map_output_dir`
  are intended to live inside the generated bundle
- use `manifest://...` to express paths relative to the bundle root

## Mapping Step

`run_mapping.sh` is intentionally mapper-agnostic. It is meant to launch an external mapper such as `lidarslam_ros2`.

The bundle spec provides `mapping.command_template`, which is rendered into the shell script with:

- `mapping_bag_path`
- `localization_bag_path`
- `reference_csv`
- `generated_map_path`
- `cloud_topic`
- `imu_topic`
- `twist_topic`

If your mapper is not available yet, you can still scaffold the bundle first and edit `run_mapping.sh` later.

## Localization Step

After the pointcloud map exists, run:

```bash
./run_localization_benchmark.sh
```

This calls:

- `ros2 run lidar_localization_ros2 benchmark_from_manifest --manifest ./localization_run_manifest.yaml`

If the spec includes a `reference:` section, run this first:

```bash
./run_extract_reference.sh
```

This extracts:

- a reference trajectory CSV
- an initial-pose parameter YAML for the localization run

The generated manifest is compatible with the existing benchmark harness and keeps:

- dataset bag path
- generated map path
- optional GT/reference CSV
- optional initial-pose YAML
- topic names
- base localization YAML
- dataset-specific parameter overrides

## Nav2 Map Step

If you also need a coarse occupancy map from the generated pointcloud map:

```bash
./run_generate_nav2_map.sh
```

When `reference_csv` is available, the helper crops around the reference route rather than rasterizing the full map.

## Reference Extraction Defaults

When `reference:` is present in the spec:

- `reference.bag_path` defaults to `dataset.localization_bag_path`
- `reference.sample_topic` defaults to `dataset.cloud_topic`
- `bundle.reference_csv` defaults to `manifest://generated/reference_pose.csv`
- `bundle.initial_pose_yaml` defaults to `manifest://generated/initial_pose.yaml`

The generated localization manifest points at those bundle-local outputs, so the intended order is:

1. `run_mapping.sh`
2. `run_extract_reference.sh`
3. `run_localization_benchmark.sh`
4. optional `run_generate_nav2_map.sh`

## Why This Workflow

This keeps the repository aligned with the current constraints:

- public urban datasets with `LiDAR + IMU + GT` often do not ship a ready-made map
- `lidar_localization_ros2` needs a pointcloud map for map-based localization
- map quality and localization quality should not be collapsed into a single same-run number

So the repo now treats `mapless public dataset` support as:

- external mapping
- internal localization benchmark
- explicit separation between the two

## Boreas Starter

For the first public `LiDAR + IMU + GT` path, use the Boreas starter workflow:

- convert raw Boreas sequences with
  `ros2 run lidar_localization_ros2 convert_boreas_sequence_to_rosbag2.py`
- scaffold a bundle from
  `param/benchmark/boreas_mapless_public_dataset_bundle.example.yaml`

See [boreas_mapless_public_dataset_workflow.md](boreas_mapless_public_dataset_workflow.md).
