# README figure generation / 首页图表生成

`generate_readme_figures.py` 从 Channel v2 准入清单绑定的六条 Tube 路径、静态地图和转向标定数据生成首页图表。

The generator renders the README figures from the six manifest-bound Channel v2 Tube paths, the static map, and the steering calibration.

## Regenerate / 重新生成

在仓库根目录运行，使用 Python 3.10 或更新版本：

```bash
python3 -m pip install matplotlib numpy Pillow PyYAML
python3 tools/visualization/generate_readme_figures.py
```

Dependencies: `matplotlib`, `numpy`, `Pillow`, `PyYAML`. The checked-in figures were generated with Python 3.12.14, matplotlib 3.11.2, numpy 2.5.4, Pillow 13.0.0, and PyYAML 6.0.3.

默认输出目录为 [`docs/assets/`](../../docs/assets)。`--output-dir` 可指定输出目录，`--repo-root` 可指定输入仓库，`--tube-asset-dir` 可指定包含准入清单的 Channel v2 资产目录。

```bash
python3 tools/visualization/generate_readme_figures.py --output-dir /tmp/lonerover-figures
```

输出文件 / Outputs:

- [`channel_tubes_v2.png`](../../docs/assets/channel_tubes_v2.png) / [SVG](../../docs/assets/channel_tubes_v2.svg)
- [`servo_pchip_calibration.png`](../../docs/assets/servo_pchip_calibration.png) / [SVG](../../docs/assets/servo_pchip_calibration.svg)
- [`readme_figure_sources.json`](../../docs/assets/readme_figure_sources.json): 输入文件的 SHA-256 和图中数据 / source hashes and extracted values

## Data sources / 数据来源

**Channel v2 Tube paths.** [`runtime/channel_v2/runtime_admission.csv`](../../src/hobot_navigation/hobot_nav/runtime/channel_v2/runtime_admission.csv) binds the six [Tube CSVs](../../src/hobot_navigation/hobot_nav/runtime/channel_v2/tubes/), with Inner, Center, and Outer paths in both CW and CCW directions. Each CSV contains 865 sampled points. The admission SHA-256 is `f1b8d865214c67f7f4536806c66f5de47b5a055918a2a5f57fcfb9e251837870`.

图中直接使用准入清单对应 CSV 的 `x_m` 和 `y_m` 坐标，按正常比例显示全图；实线表示 CW，虚线表示 CCW。地图使用 [`rdk_2026_hospital_static_1cm.yaml`](../../src/hobot_navigation/hobot_nav/maps/rdk_2026_hospital_static_1cm.yaml) 及其 PGM 文件，分辨率、原点和占据阈值均读取自 YAML。

The full-map view uses the saved coordinates directly. Solid and dashed lines identify CW and CCW. The map resolution, origin, and occupancy thresholds come from the saved YAML.

**Servo calibration.** Markers show the measured means in [`servo_left_pwm_angle.csv`](../calibration/data/servo_left_pwm_angle.csv) and [`servo_right_pwm_angle.csv`](../calibration/data/servo_right_pwm_angle.csv). Each side has seven nonzero measured means; the left records 32 total readings and the right 34, with 4–6 readings per mean. The neutral entry at PWM 1500 and 0° has `sample_count = 0` and uses a separate diamond marker.

转向曲线直接计算 [`servo_fit.c`](../calibration/servo_fit.c) 中的 `k_left_pwm_to_angle` 和 `k_right_pwm_to_angle` 冻结 PCHIP 分段系数。右侧子图以距 PWM 1500 的绝对偏移和转角绝对值展示左右不对称性。实测最大角度幅值分别为 **左转 42.0°、右转 29.4°**。

## Checks / 核对

生成时检查准入清单 SHA-256、六条 Tube 的文件哈希和坐标、地图哈希，以及冻结转向曲线与 CSV 非零实测均值之间的差值。当前舍入后的 C 系数对应的最大差值为左侧约 0.0055°、右侧约 0.0021°，均小于 0.01°。

Generation validates the admission identity, six Tube hashes and coordinates, the admitted map hash, and servo coefficient consistency against the CSV means. The JSON manifest records the exact source hashes used for each render.
