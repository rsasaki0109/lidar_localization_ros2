#!/usr/bin/env python3
"""Copy a Boreas fixture (tools/benchmark/boreas_prepare.py) with scans dropped.

From --first seconds after the first scan, every --every seconds, drops --count
consecutive /velodyne_points messages; everything else (the IMU, the map, the
reference) is kept. The copy's bag/ is new; map.ply, reference.tum and
preparation.json are symlinks to the source fixture.

usage: drop_scans.py FIXTURE OUT [--first S] [--every S] [--count N]
"""

import argparse
import json
from pathlib import Path

POINTS = "/velodyne_points"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("fixture", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--first", type=float, default=10.0)
    parser.add_argument("--every", type=float, default=10.0)
    parser.add_argument("--count", type=int, default=6)
    args = parser.parse_args()

    import rosbag2_py

    args.out.mkdir(parents=True, exist_ok=False)
    for name in ("map.ply", "reference.tum", "preparation.json"):
        (args.out / name).symlink_to((args.fixture / name).resolve())

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.fixture / "bag"), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(args.out / "bag"), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(topic)

    first_ns, next_drop_ns, left_to_drop = None, None, 0
    kept, dropped, gaps = 0, 0, []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic == POINTS:
            if first_ns is None:
                first_ns = stamp_ns
                next_drop_ns = first_ns + int(args.first * 1e9)
            if left_to_drop == 0 and stamp_ns >= next_drop_ns:
                left_to_drop = args.count
                gaps.append(round((stamp_ns - first_ns) * 1e-9, 2))
                next_drop_ns += int(args.every * 1e9)
            if left_to_drop > 0:
                left_to_drop -= 1
                dropped += 1
                continue
            kept += 1
        writer.write(topic, data, stamp_ns)
    del writer

    (args.out / "drops.json").write_text(
        json.dumps(
            {
                "source": str(args.fixture),
                "first_sec": args.first,
                "every_sec": args.every,
                "count": args.count,
                "kept_scans": kept,
                "dropped_scans": dropped,
                "gap_starts_sec": gaps,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"kept {kept}, dropped {dropped} scans in {len(gaps)} gaps: {args.out}")


if __name__ == "__main__":
    main()
