# 倒车入口 LUT 工具

这里集中放倒车进入窄通道前的 LUT 工具。LUT 把真实车模、`SmacPlannerHybrid` 可行性、footprint 和入口方向的离线搜索结果固化下来，运行时只做有限候选查询和当前 costmap 复核。

| 文件 | 作用 |
| --- | --- |
| `generate_reverse_entry_lut.py` | 调用 Nav2 Smac 生成 Reverse→Tube 入口可达性表和 witness path |
| `build_clean_reverse_entry_lut.py` | 从严格审计结果生成运行时 RGE2/RGEG2 文件 |
| `validate_reverse_entry_paths.py` | 检查保存的路径、曲率和入口连接 |
| `filter_reverse_entry_witnesses.py` | 按运行时 Gate 审计过滤 witness |
| `convert_reverse_entry_lut_v1.py` | 将旧格式 LUT 转换为自描述格式 |
| `sample_reverse_entry_starts.py` | 从已准入 LUT 采样代表性起点 |
| `materialize_lut_scenarios.py` | 从不可变 manifest 生成离线回放场景 |

常用入口：

```bash
python3 tools/reverse_entry/generate_reverse_entry_lut.py --help
python3 tools/reverse_entry/validate_reverse_entry_paths.py --help
python3 tools/reverse_entry/build_clean_reverse_entry_lut.py --help
```

生成结果应写到仓库外的输出目录。车辆尺寸、最小转弯半径、地图或舵机转角标定变化后，旧 LUT 不能直接继续使用。
