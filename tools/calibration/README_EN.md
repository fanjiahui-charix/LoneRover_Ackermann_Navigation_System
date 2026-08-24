# Calibration tools

This directory contains calibration utilities that directly affect the public navigation stack: chassis feedback, steering, and IMU calibration. They do not start or command the real vehicle. Inspect the output offline and validate it at low speed before using it on the car.

Python dependencies:

```bash
python3 -m pip install numpy pandas scipy openpyxl pyserial
```

## Servo PWM and steering angle

The race vehicle is asymmetric in left and right steering. A single linear ratio is not enough. Sweep PWM values, measure the actual steering angle on the vehicle model, and fit the two sides independently with monotone PCHIP curves:

```text
angle -> PWM       lower-controller command conversion
PWM   -> angle     offline analysis, shadow plant, and effective-angle checks
```

`fit_servo_pwm_angle_pchip.py` accepts measured CSV files or the original Excel layout and exports piecewise cubic coefficients in both directions. The CSV files under `data/` are the measured means for the published vehicle profile. `stm32_ackermann_calibration.py` and `servo_fit.c` contain the matching runtime lookup implementation.

```bash
python3 tools/calibration/fit_servo_pwm_angle_pchip.py \
  --left tools/calibration/data/servo_left_pwm_angle.csv \
  --right tools/calibration/data/servo_right_pwm_angle.csv \
  --output /tmp/servo_fit

cc -std=c99 -Wall -Wextra -pedantic -c \
  tools/calibration/servo_fit.c -o /tmp/servo_fit.o
```

After changing the servo, horn, lower-controller board, or mechanical linkage, remeasure the center PWM, both limits, and the actual angle. Steering calibration affects Ackermann curvature, the `SmacPlannerHybrid` minimum turning radius, reverse-entry LUTs, Tube paths, and the chassis model used by odometry.

## Six-face IMU and gyro bias

![Six-face IMU calibration](../../docs/images/imu_six_face_calibration.svg)

`collect_imu_six_face.py` collects raw data from the lower-controller serial port, and `filter_accel_samples.py` removes samples with an invalid gravity norm or a weak principal axis. `calibrate_imu_six_face.py` reads a static CSV with `face,ax,ay,az` columns and also accepts the `pose` column used by the collector. Label the six orientations `+x,-x,+y,-y,+z,-z`. The default ellipsoid-affine fit produces the full `acc_ta` and `acc_ba` form used by `origincarpro_base`:

```text
acc_calibrated = acc_ta @ (acc_measured - acc_ba)
```

Use `--model diagonal` when only per-axis bias and scale are wanted. If the CSV contains raw LSB values, use `--accel-scale 16384 --gravity 1.0` to fit in g units. To copy results into the current `origincarpro_base.yaml`, use the vehicle's matching scale and prepare the samples in `m/s^2` first.

```bash
python3 tools/calibration/calibrate_imu_six_face.py \
  --input six_face_samples.csv \
  --output /tmp/imu_calibration.yaml

python3 tools/calibration/collect_imu_six_face.py \
  --port /dev/ttyACM0 --baud 115200 \
  --duration 60 --output accel_6pose.csv

python3 tools/calibration/filter_accel_samples.py \
  --input accel_6pose.csv \
  --output accel_6pose_filtered.csv

python3 tools/calibration/estimate_gyro_bias.py \
  --input stationary_imu.csv \
  --output /tmp/gyro_bias.yaml

python3 tools/calibration/collect_gyro_bias.py \
  --port /dev/ttyACM0 --baud 115200 \
  --seconds 120 --output stationary_imu.csv
```

Collect gyro bias while the vehicle is completely still. Do not subtract the same bias twice: offline bias, startup zeroing, dead zones, and lower-controller forced-zero behavior must be checked as one data path.

## Relationship to navigation tuning

A practical order is: wheel direction and scale -> PWM/actual steering angle -> IMU axes, scale, and bias -> EKF -> global path -> local tracking and velocity control. If the calibration is wrong, the Ackermann model used by the planner is not the vehicle that is actually driving; tuning MPPI or RPP then only hides a lower-level error.
