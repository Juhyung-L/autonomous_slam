# autonomous_slam
This ROS2 node takes 2D costmap as input and generates waypoints to unexplored areas of the map.

Inner workings of the algorithm:
- BFS is used to find "frontier cells". These are cells with at least 1 neighboring cell with unknown occupancy.
- All connected frontier cells are grouped into and island.
- The robot travels towards the centroid of the frontier island
- Frontier islands with the largest mass and shortest distance from the robot are visited first (heuristics)

In the video, all frontier cells are colored either green or blue. Green cells mean frontier islands that the robot is currently travelling towards. Blue cells mean frontier island that the robot will visit later.
[![Watch the video](https://img.youtube.com/vi/m9cCuDcngxk/maxresdefault.jpg)](https://youtu.be/m9cCuDcngxk)
