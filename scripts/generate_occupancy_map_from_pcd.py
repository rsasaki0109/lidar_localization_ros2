#!/usr/bin/env python3

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a coarse Nav2 occupancy map (PGM + YAML) from a 3D PCD map."
    )
    parser.add_argument("--pcd", required=True, help="Input PCD map path")
    parser.add_argument("--output-dir", required=True, help="Directory for output files")
    parser.add_argument(
        "--map-name",
        default="occupancy_map",
        help="Base name for output files, without extension",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.2,
        help="Occupancy map resolution in meters per cell",
    )
    parser.add_argument(
        "--obstacle-height-m",
        type=float,
        default=0.4,
        help="Mark a cell occupied when max_z - min_z exceeds this height",
    )
    parser.add_argument(
        "--min-points-per-cell",
        type=int,
        default=2,
        help="Minimum number of points required for a cell to be considered observed",
    )
    parser.add_argument(
        "--inflate-radius-m",
        type=float,
        default=0.6,
        help="Inflate occupied cells by this radius in meters",
    )
    parser.add_argument(
        "--padding-m",
        type=float,
        default=5.0,
        help="Add this padding around the projected point extents",
    )
    parser.add_argument(
        "--reference-csv",
        default="",
        help="Optional reference trajectory CSV used to crop the occupancy map around a route",
    )
    parser.add_argument(
        "--route-padding-m",
        type=float,
        default=20.0,
        help="Padding around the reference trajectory bounds when --reference-csv is provided",
    )
    parser.add_argument(
        "--ground-band-m",
        type=float,
        default=1.0,
        help="Only classify cells whose minimum z is within this band above the route ground height",
    )
    parser.add_argument("--x-min", type=float, default=None, help="Optional explicit crop minimum x")
    parser.add_argument("--x-max", type=float, default=None, help="Optional explicit crop maximum x")
    parser.add_argument("--y-min", type=float, default=None, help="Optional explicit crop minimum y")
    parser.add_argument("--y-max", type=float, default=None, help="Optional explicit crop maximum y")
    return parser.parse_args()


def load_points(pcd_path: Path) -> np.ndarray:
    point_cloud = o3d.io.read_point_cloud(str(pcd_path))
    points = np.asarray(point_cloud.points, dtype=np.float32)
    if points.size == 0:
        raise RuntimeError(f"Point cloud is empty: {pcd_path}")
    return points


def build_observation_grids(
    points: np.ndarray,
    resolution: float,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))
    if width <= 0 or height <= 0:
        raise RuntimeError("Computed non-positive occupancy map dimensions")

    ix = np.clip(((points[:, 0] - min_x) / resolution).astype(np.int32), 0, width - 1)
    iy = np.clip(((points[:, 1] - min_y) / resolution).astype(np.int32), 0, height - 1)
    flat = iy * width + ix
    order = np.argsort(flat, kind="mergesort")
    flat_sorted = flat[order]
    z_sorted = points[order, 2]

    unique_flat, start_idx, counts = np.unique(
        flat_sorted,
        return_index=True,
        return_counts=True,
    )

    min_z = np.full(width * height, np.inf, dtype=np.float32)
    max_z = np.full(width * height, -np.inf, dtype=np.float32)
    min_z[unique_flat] = np.minimum.reduceat(z_sorted, start_idx)
    max_z[unique_flat] = np.maximum.reduceat(z_sorted, start_idx)

    count_grid = np.zeros(width * height, dtype=np.int32)
    count_grid[unique_flat] = counts

    return (
        min_z.reshape(height, width),
        max_z.reshape(height, width),
        count_grid.reshape(height, width),
        min_x,
        min_y,
    )


def determine_bounds(
    args: argparse.Namespace, points: np.ndarray
) -> tuple[float, float, float, float, str, float | None]:
    if args.reference_csv:
        xs = []
        ys = []
        zs = []
        with Path(args.reference_csv).open("r", encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                xs.append(float(row["position_x"]))
                ys.append(float(row["position_y"]))
                zs.append(float(row["position_z"]))
        if not xs or not ys:
            raise RuntimeError(f"No positions found in reference CSV: {args.reference_csv}")
        return (
            min(xs) - args.route_padding_m,
            max(xs) + args.route_padding_m,
            min(ys) - args.route_padding_m,
            max(ys) + args.route_padding_m,
            "reference_csv",
            float(np.median(np.asarray(zs, dtype=np.float32))),
        )

    if None not in (args.x_min, args.x_max, args.y_min, args.y_max):
        return (
            float(args.x_min),
            float(args.x_max),
            float(args.y_min),
            float(args.y_max),
            "manual_bounds",
            None,
        )

    return (
        float(points[:, 0].min()) - args.padding_m,
        float(points[:, 0].max()) + args.padding_m,
        float(points[:, 1].min()) - args.padding_m,
        float(points[:, 1].max()) + args.padding_m,
        "full_map",
        None,
    )


def inflate_mask(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return mask
    inflated = mask.copy()
    occupied_y, occupied_x = np.nonzero(mask)
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            yy = occupied_y + dy
            xx = occupied_x + dx
            valid = (yy >= 0) & (yy < mask.shape[0]) & (xx >= 0) & (xx < mask.shape[1])
            inflated[yy[valid], xx[valid]] = True
    return inflated


def write_map(
    output_dir: Path,
    map_name: str,
    occupancy: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    stats: dict,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = output_dir / f"{map_name}.pgm"
    yaml_path = output_dir / f"{map_name}.yaml"
    stats_path = output_dir / f"{map_name}_stats.json"

    image = Image.fromarray(occupancy, mode="L")
    image.save(pgm_path)

    yaml_data = {
        "image": pgm_path.name,
        "resolution": resolution,
        "origin": [origin_x, origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "mode": "trinary",
    }
    yaml_path.write_text(
        "\n".join(f"{key}: {json.dumps(value) if isinstance(value, str) else value}" for key, value in yaml_data.items())
        + "\n",
        encoding="utf-8",
    )
    stats_path.write_text(json.dumps(stats, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    pcd_path = Path(args.pcd).resolve()
    output_dir = Path(args.output_dir).resolve()

    points = load_points(pcd_path)
    min_x, max_x, min_y, max_y, bounds_mode, route_ground_z = determine_bounds(args, points)
    points = points[
        (points[:, 0] >= min_x)
        & (points[:, 0] <= max_x)
        & (points[:, 1] >= min_y)
        & (points[:, 1] <= max_y)
    ]
    if points.size == 0:
        raise RuntimeError("No points remain after applying the requested crop bounds")
    min_z, max_z, count_grid, origin_x, origin_y = build_observation_grids(
        points=points,
        resolution=args.resolution,
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
    )

    observed = count_grid >= args.min_points_per_cell
    if route_ground_z is not None:
        height, width = count_grid.shape
        ix = np.clip(((points[:, 0] - origin_x) / args.resolution).astype(np.int32), 0, width - 1)
        iy = np.clip(((points[:, 1] - origin_y) / args.resolution).astype(np.int32), 0, height - 1)

        low_count_grid = np.zeros((height, width), dtype=np.int32)
        high_count_grid = np.zeros((height, width), dtype=np.int32)

        low_mask = points[:, 2] <= route_ground_z + args.ground_band_m
        high_mask = points[:, 2] >= route_ground_z + args.obstacle_height_m
        np.add.at(low_count_grid, (iy[low_mask], ix[low_mask]), 1)
        np.add.at(high_count_grid, (iy[high_mask], ix[high_mask]), 1)

        ground_observed = low_count_grid >= args.min_points_per_cell
        occupied = ground_observed & (high_count_grid >= args.min_points_per_cell)
        free = ground_observed & (high_count_grid == 0)
    else:
        ground_observed = observed
        occupied = ground_observed & ((max_z - min_z) >= args.obstacle_height_m)
        free = ground_observed & ~occupied

    radius_cells = int(math.ceil(args.inflate_radius_m / args.resolution))
    occupied = inflate_mask(occupied, radius_cells)
    free = free & ~occupied

    occupancy = np.full(observed.shape, 205, dtype=np.uint8)
    occupancy[free] = 254
    occupancy[occupied] = 0
    occupancy = np.flipud(occupancy)

    stats = {
        "pcd_path": str(pcd_path),
        "point_count": int(points.shape[0]),
        "resolution_m": args.resolution,
        "width": int(occupancy.shape[1]),
        "height": int(occupancy.shape[0]),
        "origin_x": origin_x,
        "origin_y": origin_y,
        "bounds_mode": bounds_mode,
        "route_ground_z": route_ground_z,
        "ground_band_m": args.ground_band_m,
        "x_min": min_x,
        "x_max": max_x,
        "y_min": min_y,
        "y_max": max_y,
        "observed_cell_count": int(observed.sum()),
        "ground_observed_cell_count": int(ground_observed.sum()),
        "occupied_cell_count": int((occupancy == 0).sum()),
        "free_cell_count": int((occupancy == 254).sum()),
        "unknown_cell_count": int((occupancy == 205).sum()),
        "obstacle_height_m": args.obstacle_height_m,
        "min_points_per_cell": args.min_points_per_cell,
        "inflate_radius_m": args.inflate_radius_m,
        "padding_m": args.padding_m,
    }

    write_map(
        output_dir=output_dir,
        map_name=args.map_name,
        occupancy=occupancy,
        resolution=args.resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        stats=stats,
    )

    print(f"pgm: {output_dir / (args.map_name + '.pgm')}")
    print(f"yaml: {output_dir / (args.map_name + '.yaml')}")
    print(f"stats: {output_dir / (args.map_name + '_stats.json')}")
    print(json.dumps(stats, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
