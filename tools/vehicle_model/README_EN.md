# Virtual vehicle and Ackermann model

This directory contains vehicle models and replay tools decoupled from the real chassis. They publish only virtual-vehicle state and never send `/cmd_vel_safe` to the physical car.

| File | Purpose |
| --- | --- |
| `ackermann_vehicle_model.py` | vehicle geometry, steering lookup, and yaw-rate conversion |
| `ackermann_vehicle_simulator.py` | Ackermann simulator with speed/steering response, delay, and limits |
| `nav2_virtual_vehicle_replay.py` | feed a virtual chassis back to the native Nav2 chain |
| `offline_vehicle_response_sim.py` | estimate command delay and vehicle response from a user-provided rosbag |
| `test_ackermann_vehicle_simulator.py` | lightweight model and geometry tests |

Common entry points:

```bash
python3 tools/vehicle_model/test_ackermann_vehicle_simulator.py
python3 tools/vehicle_model/nav2_virtual_vehicle_replay.py --help
python3 tools/vehicle_model/offline_vehicle_response_sim.py --help
```

`nav2_virtual_vehicle_replay.py` needs a ROS 2/Nav2 environment. `offline_vehicle_response_sim.py` needs a rosbag supplied by the user. After changing the real vehicle parameters, update `ackermann_vehicle_model.py` and the simulator profile before low-speed physical validation.
