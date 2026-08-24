# Navigation tools

`tools/` contains standalone utilities for tuning and validating the public
navigation core. They do not start a private task layer or a device driver.

| Area | Entry points | Purpose |
| --- | --- | --- |
| Ackermann shadow plant | `ackermann_shadow_plant.py`, `test_ackermann_shadow_plant.py` | model steering, acceleration and command delay without driving the car |
| EKF/LiDAR analysis | `analyze_ekf_bags.py`, `analyze_lidar_landmarks.py` | inspect user-supplied recordings and sensor geometry |
| Tube/RPP tuning | `channel_tuning/` | generate, audit and evaluate channel paths and side choices |
| Reverse-entry LUT | `generate_reverse_gate_lut.py`, `generate_clean_reverse_gate_lut.py`, `validate_reverse_gate_paths.py` | build and validate finite Ackermann-feasible candidates |
| Native shadow replay | `nav2_native_shadow_replay.py`, `offline_mppi_shadow_sim.py` | run or score motor-disabled navigation replay |
| Command analysis | `analyze_command_envelope.py`, `plot_limiter_ab.py` | inspect speed, curvature, acceleration and limiter behavior |
| Base calibration | `stm32_ackermann_calibration.py` | check lower-controller steering and wheel calibration |
| Servo/IMU calibration | `calibration/` | fit PWM-to-angle curves, six-face IMU, gyro bias, and serial collection |

The virtual vehicle is deliberately separate from the real chassis. A useful
workflow is:

```text
measured vehicle response
        -> Ackermann shadow plant
        -> Tube/LUT geometry and clearance checks
        -> native Nav2 shadow replay with motor output disabled
        -> low-speed real-vehicle verification
```

Commands that consume rosbag data take the bag path explicitly. Bags, logs,
private hostnames and generated result directories are not part of this
repository.

```bash
python3 tools/ackermann_shadow_plant.py --help
python3 tools/channel_tuning/generate_channel_tubes_v2.py --help
python3 tools/generate_reverse_gate_lut.py --help
python3 tools/nav2_native_shadow_replay.py --help
python3 tools/calibration/calibrate_imu_six_face.py --help
python3 tools/calibration/fit_servo_pwm_angle_pchip.py --help
```

See [`tools/calibration/README.md`](calibration/README.md) for the Chinese
workflow and [`tools/calibration/README_EN.md`](calibration/README_EN.md) for
the English workflow.
