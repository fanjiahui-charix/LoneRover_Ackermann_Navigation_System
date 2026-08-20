# Channel Tube assets

这里保存与地图、车辆 footprint 和最小转弯半径绑定的六条通道 Tube 路径。三种横向选择分别是 `inner`、`center`、`outer`，每种选择都有 `cw` 和 `ccw` 两个方向：

| Lane | Clockwise | Counter-clockwise |
|---|---|---|
| inner | `tube_inner_cw.csv` | `tube_inner_ccw.csv` |
| center | `tube_center_cw.csv` | `tube_center_ccw.csv` |
| outer | `tube_outer_cw.csv` | `tube_outer_ccw.csv` |

它们是路径资产，不是普通导航点。修改地图、车辆尺寸、转向标定或通道安全余量后，应重新运行 `tools/channel_tuning/` 中的几何检查工具。
