#!/usr/bin/env python3
"""Copy a bag and add odom -> base TF from a TUM odometry trajectory.

TF messages are written `--lead` seconds before their stamp so that the
transform at each scan time is already available when the scan arrives
(odometry delivered without latency). usage:
  add_odom_tf.py <in_bag> <odom.tum> <out_bag> [--odom odom] [--base livox_frame] [--lead 0.3]
"""

import argparse
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

ap = argparse.ArgumentParser()
ap.add_argument("in_bag")
ap.add_argument("odom_tum")
ap.add_argument("out_bag")
ap.add_argument("--odom", default="odom")
ap.add_argument("--base", default="livox_frame")
ap.add_argument("--lead", type=float, default=0.3)
args = ap.parse_args()

ts = get_typestore(Stores.ROS2_HUMBLE)
TF = ts.types["tf2_msgs/msg/TFMessage"]
TS = ts.types["geometry_msgs/msg/TransformStamped"]
Tr = ts.types["geometry_msgs/msg/Transform"]
V3 = ts.types["geometry_msgs/msg/Vector3"]
Q = ts.types["geometry_msgs/msg/Quaternion"]
Hdr = ts.types["std_msgs/msg/Header"]
Time = ts.types["builtin_interfaces/msg/Time"]

odom = np.loadtxt(args.odom_tum)
odom = odom[np.argsort(odom[:, 0])]
tf_msgs = []
for row in odom:
    sec = int(row[0])
    nsec = int(round((row[0] - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec, nsec = sec + 1, nsec - 1_000_000_000
    t = TS(
        header=Hdr(stamp=Time(sec=sec, nanosec=nsec), frame_id=args.odom),
        child_frame_id=args.base,
        transform=Tr(
            translation=V3(x=row[1], y=row[2], z=row[3]),
            rotation=Q(x=row[4], y=row[5], z=row[6], w=row[7]),
        ),
    )
    tf_msgs.append(
        (
            int((row[0] - args.lead) * 1e9),
            ts.serialize_cdr(TF(transforms=[t]), TF.__msgtype__),
        )
    )

with (
    Reader(Path(args.in_bag)) as reader,
    Writer(Path(args.out_bag), version=8) as writer,
):
    conns = {
        c.id: writer.add_connection(c.topic, c.msgtype, typestore=ts)
        for c in reader.connections
    }
    tf_conn = writer.add_connection("/tf", TF.__msgtype__, typestore=ts)
    i = 0
    for c, t, raw in reader.messages():
        while i < len(tf_msgs) and tf_msgs[i][0] <= t:
            writer.write(tf_conn, tf_msgs[i][0], tf_msgs[i][1])
            i += 1
        writer.write(conns[c.id], t, raw)
    for stamp, raw in tf_msgs[i:]:
        writer.write(tf_conn, stamp, raw)
print("tf messages", len(tf_msgs))
