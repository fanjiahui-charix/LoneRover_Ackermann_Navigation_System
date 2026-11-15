# README figure generation / 首页图表生成

`generate_readme_figures.py` 使用六条 Tube 路径、通道避障 LUT、倒车入口 LUT、静态地图和舵机标定数据生成首页图表。

## Regenerate / 重新生成

在仓库根目录运行，使用 Python 3.10 或更新版本：

```bash
python3 -m pip install matplotlib numpy Pillow PyYAML
python3 tools/visualization/generate_readme_figures.py
```

默认输出到 [`docs/assets/`](../../docs/assets)。`--output-dir` 指定输出目录，`--repo-root` 指定输入仓库，`--tube-asset-dir` 指定通道资产目录。

Outputs:

- [`navigation_map_and_lut.png`](../../docs/assets/navigation_map_and_lut.png) / [SVG](../../docs/assets/navigation_map_and_lut.svg)
- [`servo_pchip_calibration.png`](../../docs/assets/servo_pchip_calibration.png) / [SVG](../../docs/assets/servo_pchip_calibration.svg)

## Data sources / 数据来源

**Tube paths.** [通道路径清单](../../src/hobot_navigation/hobot_nav/runtime/channel_v2/runtime_admission.csv) 选择六条 [Tube CSV](../../src/hobot_navigation/hobot_nav/runtime/channel_v2/tubes/)。图中直接使用 `x_m` 和 `y_m` 坐标；实线表示 CW，虚线表示 CCW。地图使用 [1 cm 地图 YAML](../../src/hobot_navigation/hobot_nav/maps/rdk_2026_hospital_static_1cm.yaml) 及其 PGM 文件。

**Obstacle LUT.** 左图叠加[顺时针通道避障查找表](../../src/hobot_navigation/hobot_nav/runtime/channel_v2/avoid/channel_cone_avoid_lane_1cm_cw.bin)。内侧锥桶请求 Outer Tube，外侧锥桶请求 Inner Tube；彩色区域表示锥桶的位置类别。

**Reverse-entry LUT.** 右图读取[倒车入口查找表](../../src/hobot_navigation/hobot_nav/runtime/reverse_gate_entry_clean.bin) 和[对应目标表](../../src/hobot_navigation/hobot_nav/runtime/reverse_gate_goal_id_clean.bin)，按 x/y 位置显示接纳的航向数量。

**Servo calibration.** 散点来自 [`servo_left_pwm_angle.csv`](../calibration/data/servo_left_pwm_angle.csv) 和 [`servo_right_pwm_angle.csv`](../calibration/data/servo_right_pwm_angle.csv) 中的测量均值；曲线读取 [`servo_fit.c`](../calibration/servo_fit.c) 的 PCHIP 分段系数。PWM 1500、0° 的中立参考点单独使用菱形标记。
