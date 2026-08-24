# Navigation tools

`tools/` contains standalone utilities for tuning and validating the public
navigation core. They do not start a private task layer or a device driver.

The directories are organized by responsibility: [`vehicle_model/`](vehicle_model/README_EN.md) for the Ackermann model and virtual-vehicle replay, [`reverse_entry/`](reverse_entry/README_EN.md) for reverse-entry LUTs, [`channel_tuning/`](channel_tuning/README.md) for Tube/RPP assets, [`calibration/`](calibration/README_EN.md) for lower-controller calibration, and [`offline_analysis/`](offline_analysis/README_EN.md) for rosbag and run analysis.

| Area | Entry points | Purpose |
| --- | --- | --- |
| Ackermann vehicle simulator | `vehicle_model/ackermann_vehicle_simulator.py`, `vehicle_model/test_ackermann_vehicle_simulator.py` | model steering, acceleration and command delay without driving the car |
| EKF/LiDAR analysis | `offline_analysis/analyze_ekf_bags.py`, `offline_analysis/analyze_lidar_landmarks.py` | inspect user-supplied recordings and sensor geometry |
| Tube/RPP tuning | `channel_tuning/` | generate, audit and evaluate channel paths and side choices |
| Reverse-entry LUT | `reverse_entry/` | build, clean, sample, and validate finite Ackermann-feasible candidates |
| Virtual-vehicle replay | `vehicle_model/nav2_virtual_vehicle_replay.py`, `vehicle_model/offline_vehicle_response_sim.py` | run or score motor-disabled navigation replay |
| Command analysis | `offline_analysis/analyze_command_envelope.py`, `offline_analysis/plot_limiter_ab.py` | inspect speed, curvature, acceleration and limiter behavior |
| Ackermann vehicle model | `vehicle_model/ackermann_vehicle_model.py` | provide geometry, steering lookup, and response checks |
| Servo/IMU calibration | `calibration/` | fit PWM-to-angle curves, six-face IMU, gyro bias, and serial collection |

The virtual vehicle is deliberately separate from the real chassis. A useful
workflow is:

```text
measured vehicle response
        -> Ackermann vehicle-response simulator
        -> Tube/LUT geometry and clearance checks
        -> native Nav2 virtual-vehicle replay with motor output disabled
        -> low-speed real-vehicle verification
```

Commands that consume rosbag data take the bag path explicitly. Bags, logs,
private hostnames and generated result directories are not part of this
repository.

```bash
python3 tools/vehicle_model/test_ackermann_vehicle_simulator.py
python3 tools/channel_tuning/generate_tube_paths.py --help
python3 tools/reverse_entry/generate_reverse_entry_lut.py --help
python3 tools/vehicle_model/nav2_virtual_vehicle_replay.py --help
python3 tools/calibration/calibrate_imu_six_face.py --help
python3 tools/calibration/fit_servo_pwm_angle_pchip.py --help
```

See [`tools/calibration/README.md`](calibration/README.md) for the Chinese
workflow and [`tools/calibration/README_EN.md`](calibration/README_EN.md) for
the English workflow.
