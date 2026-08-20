# Static map

The public navigation package loads an existing static map through
`nav2_map_server`. It does not include an online mapping workflow.

The checked-in 1 cm map is a navigation and tuning asset. Its resolution is a
runtime trade-off: 5 mm and 1 mm variants were tested offline, but the finer
grids increased memory use and planning cost on the target computer without
providing useful precision beyond the vehicle footprint and sensor error.

Use `static_map.launch.py` to publish the map, or start the complete public
stack with `navigation_core.launch.py`.
