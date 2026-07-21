import runpy
from pathlib import Path


MATRIX = runpy.run_path(
    str(
        Path(__file__).resolve().parents[1]
        / "scripts/run_koide_glil_tightly_coupled_matrix.py"
    ),
    run_name="koide_glil_matrix_test",
)


def test_four_sequence_acceptance_order_and_durations():
    sequences = MATRIX["SEQUENCES"]
    assert list(sequences) == [
        "outdoor_hard_01a",
        "outdoor_hard_01b",
        "outdoor_hard_02a",
        "outdoor_hard_02b",
    ]
    assert [sequence.duration_sec for sequence in sequences.values()] == [
        380.0, 302.0, 363.0, 298.0,
    ]


def test_each_sequence_has_finite_bootstrap_seed():
    for sequence in MATRIX["SEQUENCES"].values():
        assert len(sequence.center) == 3
        assert all(abs(value) < 1000.0 for value in sequence.center)
        assert -180.0 <= sequence.yaw_deg <= 180.0
