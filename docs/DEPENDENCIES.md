# Dependencies and builds

The navigation stack uses ROS 2 Humble and Nav2. Build on a ROS 2 host with:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to hobot_nav
source install/local_setup.bash
```

For the competition Supervisor and command limiter, enable their CMake target:

```bash
colcon build --packages-up-to hobot_nav \
  --cmake-args -DBUILD_COMPETITION_SUPERVISOR=ON
```

The `hbmem_wechatcode_msgs` and `ollama_msgs` interfaces are included under `src/hobot_msgs/`. Colcon builds them before `hobot_nav`. The Supervisor also links ROS `rclcpp_action`, `lifecycle_msgs`, `tf2`, `tf2_ros`, and the existing navigation message packages.

## Vehicle runtime

`navigation_core.launch.py` uses an external `LaserScan` publisher on `/scan_raw`. `competition_runtime.launch.py` uses the original X5 deployment chain and the following installed runtime packages:

| Packages | Role |
| --- | --- |
| `n10_driver` | N10 serial driver and `config/n10.yaml` |
| `hobot_shm` | platform shared-memory transport configuration |
| `hobot_usb_cam`, `hobot_codec` | camera capture and image format conversion |
| `hbmem_wechatcode` | QR recognition |
| `board_crop` | stage-triggered VLM image pipeline |
| `hobot_hdmi` | vehicle display |
| `mycar_localization`, `hobot_bev_seg` | BEV localization, semantic map and model |
| `foxglove_bridge` | optional remote inspection |

Install the matching vehicle workspace packages and provide the camera calibration and BEV model/semantic map. The admitted channel bundle is installed with `hobot_nav`:

```bash
CHANNEL_V2_DIR="$(ros2 pkg prefix hobot_nav)/share/hobot_nav/runtime/channel_v2"
```

Use `$CHANNEL_V2_DIR` for `channel_v2_asset_directory`, `$CHANNEL_V2_DIR/configuration/channel_runtime_tuning.yaml` for the tuning file, and `f1b8d865214c67f7f4536806c66f5de47b5a055918a2a5f57fcfb9e251837870` for `channel_v2_admission_sha256`. The run command is in [the competition guide](../src/hobot_navigation/hobot_nav/README_COMPETITION_EN.md).

## X5 cross-build

`x5_build.sh` uses an existing target sysroot and D-Robotics toolchain checkout. Set `X5_TOOLCHAIN_FILE` when the toolchain is outside the default location. Build the competition target with:

```bash
X5_BUILD_COMPETITION_SUPERVISOR=ON ./x5_build.sh --up-to hobot_nav
```

## Sources

- [ROS 2 Humble documentation](https://docs.ros.org/en/humble/)
- [Nav2](https://github.com/ros-navigation/navigation2)
- [D-Robotics robot_dev_config](https://github.com/D-Robotics/robot_dev_config)
- [Original vehicle workspace, 2026-develop](https://github.com/TarochLee/ros_workspace/tree/2026-develop)

The recovered Supervisor, action arbiter, message interfaces and competition launch/configuration originate from `TarochLee/ros_workspace` commit [`aa018d6627b272c0797bab58faaa3dd07884f2d1`](https://github.com/TarochLee/ros_workspace/tree/aa018d6627b272c0797bab58faaa3dd07884f2d1) (Apache-2.0). The `competition_*` launch/configuration variants keep that deployment separate from the existing `navigation_core.launch.py` defaults.
