# Stage and roll back map packages

The experimental `site_store.py` keeps versioned maps, occupancy grids,
and ROS parameters together. It runs on Linux with Python 3 and PyYAML.

Each package directory contains a `site.json` manifest:

```json
{
  "schema": 1,
  "roles": {
    "pointcloud": "map.ply",
    "occupancy": "grid.yaml",
    "parameters": "localizer.yaml"
  },
  "frames": {"map": "map", "odom": "odom", "base": "base_link"},
  "sha256": {
    "map.ply": "<digest>",
    "grid.yaml": "<digest>",
    "grid.pgm": "<digest>",
    "localizer.yaml": "<digest>"
  }
}
```

Compute each digest with `sha256sum`. The grid YAML references the image
with a relative path. Use the same frame names in the parameters and the
manifest. The validator checks the manifest, file hashes, the occupancy
YAML and its image reference, and the structure of the ROS parameter
file. Verify the map and parameters by starting the localizer in a test
environment. Validate map alignment on representative scans before use.

```bash
tool=experiments/localization_deployment/site_store.py
store=/path/to/site_store
python3 "$tool" --store "$store" stage v1 /path/to/site_v1
python3 "$tool" --store "$store" activate v1
python3 "$tool" --store "$store" stage v2 /path/to/site_v2
python3 "$tool" --store "$store" activate v2
python3 "$tool" --store "$store" current --with-parameters > /tmp/site-selection.json
jq -r .parameters_yaml /tmp/site-selection.json > /tmp/site-localizer.yaml
python3 "$tool" --store "$store" rollback
```

`current` returns the selected version, absolute paths and frame names as
JSON. `current --with-parameters` also includes a `parameters_yaml`
field with `map_path` set to the selected package's pointcloud. Both
the paths and the YAML come from the same selection. Pass that YAML as `localization_param_dir` and use
the matching map, occupancy grid and frame names from `current` when
launching Quickstart. Generate the YAML again after switching versions or
rolling back, and restart the localizer to apply it.

A failed staging or selection keeps the previous selection file. Rollback
validates the previous version before selecting it.

In a ROS 2 Jazzy container, Quickstart loaded the selected Boreas point
cloud in all three launches. The NDT thread count changed from 2 to 4 on
activation and returned to 2 on rollback. A package with malformed ROS
YAML and matching file hashes was rejected while v2 remained selected.
