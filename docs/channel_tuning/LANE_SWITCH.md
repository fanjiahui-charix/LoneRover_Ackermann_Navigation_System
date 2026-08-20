# Tube path switching

路径切换只在已有 Tube 和连接段之间进行。连接段需要满足端点位姿、方向、footprint 净空和最小转弯半径约束；在线选择失败时应保持当前安全路径，由上层决定后续处理。

```bash
python3 tools/channel_tuning/generate_channel_connectors_v2.py --help
python3 tools/channel_tuning/evaluate_channel_connectors.py --help
```

连接段是路径资产，不是实时生成的导航点集合。地图或车辆参数变化后必须重新验证。
