#!/usr/bin/env python3

import argparse
import csv
import os
import signal
import sys

import rospy
from hdl_localization.msg import ScanMatchingStatus
from nav_msgs.msg import Odometry


class Recorder:
    def __init__(self, output_dir: str) -> None:
        self.output_dir = os.path.abspath(output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        self.odom_index = 0
        self.status_index = 0

        odom_path = os.path.join(self.output_dir, "hdl_odom.csv")
        status_path = os.path.join(self.output_dir, "hdl_status.csv")

        self.odom_stream = open(odom_path, "w", newline="", encoding="utf-8")
        self.status_stream = open(status_path, "w", newline="", encoding="utf-8")

        self.odom_writer = csv.writer(self.odom_stream)
        self.status_writer = csv.writer(self.status_stream)

        self.odom_writer.writerow(
            [
                "message_index",
                "stamp_sec",
                "frame_id",
                "child_frame_id",
                "position_x",
                "position_y",
                "position_z",
                "orientation_x",
                "orientation_y",
                "orientation_z",
                "orientation_w",
                "linear_x",
                "linear_y",
                "linear_z",
                "angular_x",
                "angular_y",
                "angular_z",
            ]
        )
        self.status_writer.writerow(
            [
                "message_index",
                "stamp_sec",
                "frame_id",
                "has_converged",
                "matching_error",
                "inlier_fraction",
                "relative_pose_tx",
                "relative_pose_ty",
                "relative_pose_tz",
                "relative_pose_qx",
                "relative_pose_qy",
                "relative_pose_qz",
                "relative_pose_qw",
                "prediction_label_count",
                "prediction_error_count",
            ]
        )

        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=100)
        rospy.Subscriber("/status", ScanMatchingStatus, self.status_callback, queue_size=100)

    def close(self) -> None:
        self.odom_stream.close()
        self.status_stream.close()

    def odom_callback(self, msg: Odometry) -> None:
        stamp_sec = msg.header.stamp.to_sec()
        self.odom_writer.writerow(
            [
                self.odom_index,
                f"{stamp_sec:.9f}",
                msg.header.frame_id,
                msg.child_frame_id,
                f"{msg.pose.pose.position.x:.10f}",
                f"{msg.pose.pose.position.y:.10f}",
                f"{msg.pose.pose.position.z:.10f}",
                f"{msg.pose.pose.orientation.x:.10f}",
                f"{msg.pose.pose.orientation.y:.10f}",
                f"{msg.pose.pose.orientation.z:.10f}",
                f"{msg.pose.pose.orientation.w:.10f}",
                f"{msg.twist.twist.linear.x:.10f}",
                f"{msg.twist.twist.linear.y:.10f}",
                f"{msg.twist.twist.linear.z:.10f}",
                f"{msg.twist.twist.angular.x:.10f}",
                f"{msg.twist.twist.angular.y:.10f}",
                f"{msg.twist.twist.angular.z:.10f}",
            ]
        )
        self.odom_stream.flush()
        self.odom_index += 1

    def status_callback(self, msg: ScanMatchingStatus) -> None:
        stamp_sec = msg.header.stamp.to_sec()
        self.status_writer.writerow(
            [
                self.status_index,
                f"{stamp_sec:.9f}",
                msg.header.frame_id,
                int(msg.has_converged),
                f"{msg.matching_error:.10f}",
                f"{msg.inlier_fraction:.10f}",
                f"{msg.relative_pose.translation.x:.10f}",
                f"{msg.relative_pose.translation.y:.10f}",
                f"{msg.relative_pose.translation.z:.10f}",
                f"{msg.relative_pose.rotation.x:.10f}",
                f"{msg.relative_pose.rotation.y:.10f}",
                f"{msg.relative_pose.rotation.z:.10f}",
                f"{msg.relative_pose.rotation.w:.10f}",
                len(msg.prediction_labels),
                len(msg.prediction_errors),
            ]
        )
        self.status_stream.flush()
        self.status_index += 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record hdl_localization outputs into CSV files.")
    parser.add_argument("--output-dir", required=True, help="Directory to store hdl_odom.csv and hdl_status.csv")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rospy.init_node("record_hdl_localization_outputs", anonymous=False)
    recorder = Recorder(args.output_dir)

    def handle_signal(signum, frame) -> None:
        recorder.close()
        rospy.signal_shutdown(f"signal {signum}")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rospy.spin()
    finally:
        recorder.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
