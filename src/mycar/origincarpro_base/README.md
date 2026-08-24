# origincarpro_base

This package is the user-maintained ROS 2 base interface for the Ackermann
vehicle. It parses the lower-controller serial protocol, publishes raw wheel
odometry and IMU data, broadcasts the IMU and lidar static transforms, and
converts a constrained `Twist` command into the chassis protocol.

The public navigation stack expects:

| Interface | Default |
| --- | --- |
| raw wheel odometry | `/odom/data_raw` |
| raw IMU | `/imu/data_raw` |
| fused IMU | `/imu/fused/data_raw` |
| command input | `/cmd_vel_safe` in `hobot_nav/navigation_core.launch.py` |
| dynamic frame | `odom -> base_link` is owned by `ekf_fusion` |
| static frames | `base_link -> imu_link`, `base_link -> laser_link` |

The lower controller must provide calibrated wheel feedback, IMU samples with
consistent axes and timestamps, a stop-on-timeout behavior, and a protocol
that can distinguish motion commands from feedback frames. Steering PWM and
wheel scale are vehicle-specific parameters; changing the board or sensor
requires rechecking them.

## Configuration

`config/origincarpro_base.yaml` contains the current vehicle profile. Change at
least the serial device, wheel/steering calibration, IMU calibration and lidar
extrinsic before using another vehicle. The package deliberately contains no
camera, image-processing or vendor SDK integration.

Calibration utilities live in the repository-level
[`tools/calibration/`](../../../tools/calibration/README_EN.md). They cover
servo PWM/angle PCHIP fitting, six-face IMU calibration, stationary gyro bias,
and lower-controller serial collection.

## Run the base interface

```bash
ros2 run origincarpro_base origincarpro_base_node \
  --ros-args --params-file src/mycar/origincarpro_base/config/origincarpro_base.yaml
```

Run it together with EKF and Nav2 through
`ros2 launch hobot_nav navigation_core.launch.py`.
