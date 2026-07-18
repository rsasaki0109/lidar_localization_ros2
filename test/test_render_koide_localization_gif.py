import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "render_koide_localization_gif.py"
SPEC = importlib.util.spec_from_file_location("render_koide_localization_gif", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class RenderKoideLocalizationGifTest(unittest.TestCase):
    def test_alignment_actions_are_classified_for_gallery_overlay(self):
        with tempfile.TemporaryDirectory() as raw_directory:
            path = Path(raw_directory) / "alignment.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=["stamp_sec", "values_json"])
                writer.writeheader()
                for stamp, action in (
                    (1.0, "accept_measurement"),
                    (2.0, "reject_measurement"),
                    (3.0, "request_reinitialization"),
                ):
                    writer.writerow(
                        {
                            "stamp_sec": stamp,
                            "values_json": json.dumps({"recovery_action": action}),
                        }
                    )

            self.assertEqual(
                MODULE.load_alignment(path),
                [(1.0, "matched"), (2.0, "bridge"), (3.0, "request")],
            )

    def test_metrics_label_is_loaded_from_authoritative_json(self):
        with tempfile.TemporaryDirectory() as raw_directory:
            directory = Path(raw_directory)
            trajectory = directory / "trajectory.json"
            coverage = directory / "coverage.json"
            trajectory.write_text(json.dumps({"translation_rmse_m": 0.6554126}))
            coverage.write_text(
                json.dumps(
                    {
                        "output_coverage_percent": 100.0,
                        "matched_coverage_percent": 63.0,
                        "max_output_gap_sec": 0.750658,
                    }
                )
            )

            self.assertEqual(
                MODULE.load_metrics_label(trajectory, coverage, "Full run: "),
                "Full run: RMSE 0.655 m | output 100.0% | "
                "NDT matched 63.0% | max gap 0.751 s",
            )

    def test_recovery_crop_preserves_requested_time_window(self):
        poses = MODULE.np.asarray(
            [[100.0 + index, float(index), 0.0, 0.0] for index in range(11)],
            dtype=MODULE.np.float64,
        )
        cropped = MODULE.crop_poses(poses, start_offset_sec=4.0, duration_sec=3.0)
        self.assertEqual(cropped[:, 0].tolist(), [104.0, 105.0, 106.0, 107.0])


if __name__ == "__main__":
    unittest.main()
