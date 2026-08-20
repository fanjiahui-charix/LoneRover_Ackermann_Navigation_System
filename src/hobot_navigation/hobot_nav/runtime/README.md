# Runtime assets

This directory contains compact navigation assets generated offline:

- reverse-entry and reverse-goal lookup tables;
- direct-gate feasibility data;
- the cone-side classification table;
- quality and validation metadata for the checked-in assets.

They are not models, camera data, rosbag files or build outputs. Regenerate the
assets when the map, vehicle footprint, steering calibration or cone geometry
changes. The source-side generators and validators are kept under `tools/`.
