# Tube geometry

公开 Tube 由 `inner`、`center`、`outer` 三个横向选择组成，每个选择包含 `cw` 和 `ccw` 两个方向。路径与 1 cm 地图、车辆 footprint 和最小转弯半径绑定。

```bash
python3 tools/generate_channel_tubes_v2.py --help
python3 tools/channel_tuning/evaluate_channel_tubes.py --help
python3 tools/channel_tuning/visualize_channel_assets.py --help
```

先做几何和净空检查，再把候选路径交给延迟车模和实体车验证。修改地图、车体尺寸或转向标定后，应重新生成和审计路径。
