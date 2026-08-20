# Waypoints

This directory contains a small generic waypoint example for testing Nav2 on
the checked-in static map. It does not contain the private competition task
route or any sensor-driven mission logic.

`example_waypoints.yaml` uses the format consumed by `waypoint_audit.py` and
`waypoint_recorder.py`. Replace the coordinates with poses measured in your own
map, then audit the route before sending it to Nav2.

```bash
ros2 run hobot_nav waypoint_audit.py \
  --waypoints-file src/hobot_navigation/hobot_nav/waypoints/example_waypoints.yaml \
  --route-name default
```
