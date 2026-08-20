# RPP tuning

RPP 负责跟踪规划得到的 `Path`，不负责替代全局规划器。Tube 调参时优先观察横向误差、曲率段速度、前视距离、控制延迟和命令连续性。

```bash
python3 tools/channel_tuning/evaluate_channel_rpp_run.py --help
python3 tools/nav2_native_shadow_replay.py --help
python3 tools/offline_mppi_shadow_sim.py --help
```

离线评估只接收用户显式提供的路径或数据目录。参数通过虚拟车模筛选后，再在实体车上从低速开始确认。
