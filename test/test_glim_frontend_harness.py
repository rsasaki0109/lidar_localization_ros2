from pathlib import Path
import re
import unittest
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT = ROOT / "experiments" / "koide_glim_frontend_localization"
sys.path.insert(0, str(EXPERIMENT))
import glim_tf_relay


class GlimFrontendHarnessTest(unittest.TestCase):
    def test_glim_tf_relay_renames_local_odom_parent(self):
        from geometry_msgs.msg import TransformStamped

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp.sec = 7
        odom_to_base.child_frame_id = "livox_frame"
        odom_to_base.transform.translation.x = 2.0
        odom_to_base.transform.rotation.w = 1.0

        result = glim_tf_relay.rename_parent(odom_to_base, "glim_odom")
        self.assertEqual(result.header.frame_id, "glim_odom")
        self.assertEqual(result.child_frame_id, "livox_frame")
        self.assertEqual(result.header.stamp.sec, 7)
        self.assertAlmostEqual(result.transform.translation.x, 2.0, places=6)
        self.assertAlmostEqual(result.transform.translation.y, 0.0, places=6)
        self.assertEqual(odom_to_base.header.frame_id, "")

    def test_product_bridge_features_remain_default_off(self):
        component = (ROOT / "src" / "lidar_localization_component.cpp").read_text(
            encoding="utf-8"
        )
        localizer_launch = (ROOT / "launch" / "lidar_localization.launch.py").read_text(
            encoding="utf-8"
        )
        recovery_launch = (
            ROOT / "launch" / "global_localization_recovery.launch.py"
        ).read_text(encoding="utf-8")

        for parameter in (
            "use_odom_tf_prediction",
            "constrain_odom_tf_prediction_to_planar",
            "publish_bridge_pose_when_lost",
            "enable_odom_tf_prediction_correction_guard",
            "enable_odom_tf_prediction_recovery_correction_guard",
            "enable_timer_publishing",
        ):
            self.assertIn(f'declare_parameter("{parameter}", false)', component)

        for argument in (
            "use_odom_tf_prediction",
            "publish_bridge_pose_when_lost",
        ):
            pattern = rf"'{re.escape(argument)}',\s*default_value='false'"
            self.assertRegex(localizer_launch, pattern)
            self.assertRegex(recovery_launch, pattern)

        self.assertIn(
            "'supervisor_use_odom_bridge_candidate', default_value='false'",
            recovery_launch,
        )
        self.assertIn(
            "'supervisor_enable_bbs_shadow_motion_gate', default_value='false'",
            recovery_launch,
        )
        supervisor_node = (
            ROOT / "scripts" / "reinitialization_supervisor_node.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn(
            "self.state.name != rsp.STATE_VERIFYING", supervisor_node,
            "cross-check reseeds must not bypass pre-publication shadow validation",
        )

    def test_manifests_have_safe_unique_domains_and_matching_outputs(self):
        domains = set()
        manifests = sorted(EXPERIMENT.glob("*.yaml"))
        self.assertTrue(manifests)
        for path in manifests:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
            benchmark = data["benchmark"]
            command = data["system"]["command"]
            domain = int(benchmark["ros_domain_id"])
            output_dir = str(benchmark["output_dir"])

            self.assertLessEqual(domain, 101)
            self.assertGreaterEqual(domain, 0)
            self.assertNotIn(domain, domains)
            domains.add(domain)
            self.assertTrue(output_dir.startswith("/tmp/glimfrontend_runs/"))
            self.assertIn(f"ROS_DOMAIN_ID={domain}", command)
            self.assertIn(f"OUT_DIR={output_dir}", command)
            self.assertNotIn("/tmp/claude", command)

    def test_non_01a_manifests_pass_the_sequence_initial_pose(self):
        for sequence in ("01b", "02a", "02b"):
            path = EXPERIMENT / f"fullseq_glim_frontend_{sequence}.yaml"
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
            command = data["system"]["command"]
            self.assertIn(f"outdoor_hard_{sequence}_initial_pose.yaml", command)
            self.assertIn("INITIAL_POSE_YAML=", command)

    def test_wrapper_defaults_to_candidate_and_idle_suite_is_self_contained(self):
        wrapper = (EXPERIMENT / "glim_frontend_system.sh").read_text(encoding="utf-8")
        idle_suite = (EXPERIMENT / "idle_glimfrontend_suite.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn('FASTDDS_BUILTIN_TRANSPORTS:-UDPv4', wrapper)
        self.assertIn('USE_ODOM_TF_PREDICTION:-true', wrapper)
        self.assertIn('"enable_odom_tf_prediction_correction_guard": True', wrapper)
        self.assertIn('ODOM_CORRECTION_GUARD_TRANSLATION_M:-2.0', wrapper)
        self.assertIn(
            '"odom_tf_prediction_correction_guard_translation_m": '
            'odom_correction_guard_translation_m', wrapper)
        self.assertIn('LOCALIZER_SCORE_THRESHOLD:-6.0', wrapper)
        self.assertIn('ENABLE_BORDERLINE_SEED_REJECTION_GATE:-true', wrapper)
        self.assertIn('PUBLISH_BRIDGE_POSE_WHEN_LOST:-true', wrapper)
        self.assertIn('"enable_timer_publishing": enable_bridge_timer', wrapper)
        self.assertIn('"pose_publish_frequency": 12.5', wrapper)
        self.assertIn('SUPERVISOR_USE_ODOM_BRIDGE_CANDIDATE:-true', wrapper)
        component = (ROOT / "src" / "lidar_localization_component.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("odom_bridge_pose_pub_->publish(pose_copy)", component)
        self.assertNotIn("/tmp/claude", idle_suite)
        self.assertIn('fullseq_glim_frontend_${seq}.yaml', idle_suite)

    def test_recovery_window_ab_changes_only_bridge_feature_switches(self):
        baseline = yaml.safe_load(
            (EXPERIMENT / "recovery_window_glim_frontend_01a_baseline.yaml").read_text(
                encoding="utf-8"
            )
        )
        candidate = yaml.safe_load(
            (EXPERIMENT / "recovery_window_glim_frontend_01a_candidate.yaml").read_text(
                encoding="utf-8"
            )
        )

        for key in ("bag_path", "map_path", "reference_csv", "initial_pose_yaml"):
            self.assertEqual(baseline["dataset"][key], candidate["dataset"][key])
        for key in (
            "bag_duration",
            "bag_start_offset",
            "settle_seconds",
            "post_roll_seconds",
            "max_time_diff",
        ):
            self.assertEqual(baseline["benchmark"][key], candidate["benchmark"][key])

        baseline_command = baseline["system"]["command"]
        candidate_command = candidate["system"]["command"]
        for switch in (
            "USE_ODOM_TF_PREDICTION",
            "PUBLISH_BRIDGE_POSE_WHEN_LOST",
            "SUPERVISOR_USE_ODOM_BRIDGE_CANDIDATE",
        ):
            self.assertIn(f"{switch}=false", baseline_command)
            self.assertIn(f"{switch}=true", candidate_command)

        tuned = yaml.safe_load(
            (EXPERIMENT / "recovery_window_glim_frontend_01a_candidate_score10.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertIn("LOCALIZER_SCORE_THRESHOLD=10.0", tuned["system"]["command"])
        self.assertIn(
            "ENABLE_BORDERLINE_SEED_REJECTION_GATE=false", tuned["system"]["command"]
        )


if __name__ == "__main__":
    unittest.main()
