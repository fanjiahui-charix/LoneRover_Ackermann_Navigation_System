# Cone avoidance geometry

锥桶避障图将锥桶位置映射为可选的 Tube 侧向路径。它只使用雷达聚类后的几何点和静态地图，不包含视觉识别或模型推理。

```bash
python3 tools/channel_tuning/generate_channel_cone_avoid_map.py --help
python3 tools/channel_tuning/evaluate_channel_avoid_map.py --help
```

生成结果必须绑定当前地图、Tube、连接段、车辆 footprint 和锥桶有效半径。任何一个输入变化都应重新生成和评估。
