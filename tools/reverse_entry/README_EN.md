# Reverse-entry LUT tools

This directory contains the LUT tools used before reversing into the narrow channel. The LUT freezes offline checks for the real vehicle model, `SmacPlannerHybrid` feasibility, footprint, and entry heading. Runtime selection remains finite and is rechecked against the current costmap.

| File | Purpose |
| --- | --- |
| `generate_reverse_entry_lut.py` | generate Reverse→Tube reachability and witness paths with Nav2 Smac |
| `build_clean_reverse_entry_lut.py` | build runtime RGE2/RGEG2 files from strict audit results |
| `validate_reverse_entry_paths.py` | check saved paths, curvature, and entry connection |
| `filter_reverse_entry_witnesses.py` | filter witnesses using the runtime gate audit |
| `convert_reverse_entry_lut_v1.py` | convert the old LUT format to the self-describing format |
| `sample_reverse_entry_starts.py` | sample representative starts from an admitted LUT |
| `materialize_lut_scenarios.py` | materialize offline replay scenarios from an immutable manifest |

Common entry points:

```bash
python3 tools/reverse_entry/generate_reverse_entry_lut.py --help
python3 tools/reverse_entry/validate_reverse_entry_paths.py --help
python3 tools/reverse_entry/build_clean_reverse_entry_lut.py --help
```

Write generated results outside the repository. A change to vehicle dimensions, minimum turning radius, map, or steering calibration invalidates the old LUT.
