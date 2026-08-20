# lidar_perception

`lidar_perception` is a vendor-neutral ROS 2 package for 2D `LaserScan`
processing. It contains no lidar serial driver. Any external driver may feed
`/scan_raw` if it publishes valid timestamps and the configured laser frame.

The pipeline is:

```text
/scan_raw -> scan_filter_node -> /scan -> cone_detector_node
                                      -> /cones/points, /cones/poses, /cones/markers
```

It provides range and angle calibration, median/jump/speckle filtering,
optional first-order deskew using `/odom`, cone clustering with temporal
confirmation, and Nav2 costmap plugins for short-lived cone obstacles.

Build with the normal workspace build. The package uses the installed ROS 2
and Nav2 headers through `find_package`; it does not hard-code a vendor sysroot
or copy platform sources into this repository.

```bash
ros2 launch hobot_nav lidar_pipeline.launch.py \
  raw_scan_topic:=/my_lidar/scan \
  filtered_scan_topic:=/scan
```

Main parameters are in `config/lidar_perception.yaml`. Start with the measured
range scale, bias and angular offset, then tune the speckle and cluster gates.
The cone layer must be enabled in the Nav2 global/local costmap configuration
before cone detections can affect planning.
