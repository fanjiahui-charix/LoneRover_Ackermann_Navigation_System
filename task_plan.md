# Public navigation release plan

## Completed

- Define the repository as a navigation-only release.
- Keep localization, lidar geometry perception, Nav2 integration, control, maps, Tube assets, LUT tools, and the virtual vehicle.
- Remove private task code, vision/model code, platform SDK copies, recording-only packages, and the uncertain lidar driver.
- Provide separate Chinese and English root documentation.

## Verification

- Audit source and documentation for private runtime references.
- Run formatting and syntax checks for Python, launch files, YAML, CMake, and package manifests.
- Build with ROS 2 Humble/Nav2 when the environment provides those dependencies.
- Run unit tests and lightweight offline-tool checks.
- Review the final diff, commit with the configured GitHub identity, and push only after the public boundary is confirmed.

## Documentation amendment

- Move lower-controller calibration into the lower-controller requirements section.
- Document the executable PWM, six-face IMU, filtering, and gyro-bias workflow.
- Remove the six-face SVG asset and all references to it.
