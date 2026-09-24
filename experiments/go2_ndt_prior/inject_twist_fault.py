#!/usr/bin/env python3
"""Create an isolated SQLite bag copy with a bounded /leg_twist fault."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sqlite3

import yaml
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.serialization import deserialize_message, serialize_message


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--start-stamp', type=float, required=True,
                        help='absolute bag storage timestamp in seconds')
    parser.add_argument('--duration', type=float, default=4.0)
    parser.add_argument('--mode', required=True,
                        choices=['drop', 'stale', 'nan', 'uncertain_bias', 'confident_bias'])
    args = parser.parse_args()
    if not math.isfinite(args.start_stamp) or not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('start stamp must be finite and duration finite and positive')
    if args.output.exists():
        parser.error('output already exists; use a fresh directory')
    metadata = yaml.safe_load((args.bag / 'metadata.yaml').read_text())
    info = metadata['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'sqlite3' or len(info['relative_file_paths']) != 1:
        parser.error('this experiment supports a single SQLite file only')
    relative = info['relative_file_paths'][0]
    source = args.bag / relative
    source_hash = sha256(source)
    shutil.copytree(args.bag, args.output)
    target = args.output / relative
    start_ns = round(args.start_stamp * 1e9)
    end_ns = start_ns + round(args.duration * 1e9)
    with sqlite3.connect(target) as connection:
        topic = connection.execute("SELECT id FROM topics WHERE name='/leg_twist'").fetchone()
        if topic is None:
            raise ValueError('no /leg_twist in input bag')
        rows = connection.execute(
            'SELECT id, data FROM messages WHERE topic_id=? AND timestamp>=? AND timestamp<?',
            (topic[0], start_ns, end_ns)).fetchall()
        if not rows:
            raise ValueError('fault window contains no twist samples')
        for message_id, data in rows:
            if args.mode == 'drop':
                connection.execute('DELETE FROM messages WHERE id=?', (message_id,))
                continue
            msg = deserialize_message(data, TwistWithCovarianceStamped)
            if args.mode == 'stale':
                msg.header.stamp.sec -= 2
            elif args.mode == 'nan':
                msg.twist.twist.linear.x = float('nan')
            else:
                msg.twist.twist.linear.x += 1.0
                if args.mode == 'uncertain_bias':
                    msg.twist.covariance[0] = 1.0
            connection.execute('UPDATE messages SET data=? WHERE id=?',
                               (serialize_message(msg), message_id))
        counts = dict(connection.execute(
            'SELECT topics.name, COUNT(*) FROM messages JOIN topics ON topic_id=topics.id '
            'GROUP BY topic_id'))
    total = sum(counts.values())
    info['message_count'] = total
    for topic in info['topics_with_message_count']:
        topic['message_count'] = counts.get(topic['topic_metadata']['name'], 0)
    for entry in info.get('files', []):
        entry['message_count'] = total
    (args.output / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    assert sha256(source) == source_hash, 'original input changed during experiment'
    manifest = {
        'source': str(args.bag.resolve()), 'source_db_sha256': source_hash,
        'output_db_sha256': sha256(target), 'generator_sha256': sha256(Path(__file__)),
        'mode': args.mode, 'storage_window_ns': [start_ns, end_ns],
        'affected_twist_messages': len(rows), 'message_counts': counts,
        'note': 'Synthetic fault experiment; original lidar/IMU messages retained. '
                'Confident bias deliberately retains original covariance.',
    }
    (args.output / 'fault_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
