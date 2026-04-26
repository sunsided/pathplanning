# pathplanning

Path planning playground on [Berlin OSM (2026-04-25)](https://download.geofabrik.de/europe/germany/berlin.html).

![A* Animation](readme/video.webp)

## Directed Graph

Planning is on the directed graph of the Berlin road network.

| A* | A* (reverse) |
|---|---|
|![A* pathfinding from north to south on Berlin OSM map with blue route highlighted and green start marker](readme/astar-one.png)|![A* pathfinding from south to north on Berlin OSM map with blue route highlighted and pink end marker](readme/astar-two.png)|

## Cost Base

Different cost functions can be selected in addition to planner heuristics:

### Shortest Time

![A* with shortest time cost](readme/time.png)

### Shortest Distance

![A* with shortest distance cost](readme/dist.png)
