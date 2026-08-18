from __future__ import annotations

from experiments.corridor_structure_gate.interface import StructureGateInput
from experiments.corridor_structure_gate.interface import StructureGateResult


class ZScoreOnly:
    """Baseline: accept every candidate (2D BBS order is unchanged).

    This reproduces the documented north/south corridor alias failure: a 2D
    occupancy score cannot tell structurally different but footprint-identical
    locations apart, so along-corridor aliases stay in the top-K.
    """

    name = "z_score_only"
    design = "baseline, no 3D structure evidence"

    def apply(self, gate_input: StructureGateInput) -> StructureGateResult:
        return StructureGateResult(
            similarities=[1.0] * len(gate_input.candidates),
            accepted=[True] * len(gate_input.candidates),
            reason="baseline accepts all candidates",
        )
