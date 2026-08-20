# Channel modes

The narrow-channel assets use three map-frame Tube paths in each travel
direction: inner, center and outer. The paths are precomputed from the vehicle
footprint, turning-radius limit and channel geometry. A runtime selector can
choose one of the finite paths according to current clearance and progress;
RPP then tracks the selected path continuously.

This was more stable than stacking many manually selected waypoints or asking
a general local planner to rediscover the whole narrow route at runtime. The
Tube is the geometric path asset; RPP is the tracker. Obstacle points still
enter the local costmap and can reject or switch a candidate.

The reverse-entry LUTs use the real vehicle geometry and planner constraints to
turn a continuous pose search into a bounded lookup. They are navigation data,
not recognition data. Regenerate them after changing the map, footprint,
steering calibration, minimum turning radius or obstacle model.
