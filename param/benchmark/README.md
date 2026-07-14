# Benchmark parameter inputs

This directory holds reproducible experiment inputs, not recommended runtime
presets. Keep an input after a run when it is needed to reproduce a documented
comparison or release boundary.

## File kinds

| Form | Purpose |
| --- | --- |
| `*.yaml` with `/**` → `ros__parameters` | Frozen localizer configuration |
| `*_manifest*.yaml` or `*.example.yaml` | Dataset/run orchestration input |
| `*_compare.json` | Baseline/candidate comparison specification |
| `*_sweep.json` | Parameter sweep specification |

The scripts consuming each schema are the source of truth. Most run manifests
are handled by `benchmark_from_manifest`; comparison specifications are handled
by `benchmark_compare_runs` or a named experiment script.

## Naming

Use lowercase snake case and encode only information needed to identify the
fixture:

```text
<dataset>_<sequence-or-window>_<backend-or-feature>_<purpose>.<yaml|json>
```

Prefer descriptive feature names over unexplained revision numbers. If a
revision suffix is required for historical reproduction, explain it in the
related experiment README or validation log.

## Lifecycle

- Put general runtime presets in the parent directory only after validation.
- Do not place generated maps, bags, logs, trajectories, or dashboards here.
- Use `/tmp` or `artifacts/` for generated run outputs.
- Preserve published comparison inputs; add a successor instead of silently
  changing the meaning of a historical fixture.
- Keep private dataset paths out of committed files. Use `*.example.yaml` for
  templates that require local paths.
