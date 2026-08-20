# Building the public navigation packages

本目录只提供导航集成层和公开的自有包。ROS 2 Humble、Nav2、底盘平台包和外部雷达驱动由目标环境提供。

## Native build

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ros_workspace_open_source
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
```

## X5 cross build

`x5_build.sh` 只接受仓库外部的工具链目录。通过 `X5_TOOLCHAIN_ROOT` 或命令行参数指定平台环境，不要把系统 SDK、厂商包或工具链复制回公开仓库。

```bash
X5_TOOLCHAIN_ROOT=/path/to/external/toolchain ./x5_build.sh --select hobot_nav
```

具体依赖见仓库根目录 [`docs/DEPENDENCIES.md`](../../docs/DEPENDENCIES.md)。公开版本不包含建图流程，也不包含设备驱动源码。
