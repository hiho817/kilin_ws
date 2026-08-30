# Local terrain mapping

`local_terrain_window` converts FAST-LIO registered clouds into the bounded
`/kilin/terrain/local_window` elevation grid consumed by the live planner.

## Bounded retained terrain

Only points first observed in the configured forward ROI can contribute to the
terrain map.  They are immediately reduced to one robust height per fixed
map-aligned grid cell; raw scans are not retained.  The cache is removed when
a cell leaves the current planner window (plus the configured margin), or when
its observation age exceeds `retained_terrain.max_age_s` (45 s by default).

This preserves an observed ramp exit across a short LiDAR occlusion while
keeping estimator memory and publish work proportional to the local window,
not to experiment duration.  A single conflicting height requires
`temporal_cells.replacement_observations` consistent observations before it can
replace a cached cell.

For the current real-Kilin setup, use the default fixed 0.10 m grid and 5 Hz
publish rate.  Inspect `/kilin/terrain/local_window/cells` in RViz.  The map is
deliberately local: unknown cells remain unknown rather than being filled from
a global accumulated cloud.
