# Hybrid A* C++ for ROS 2

A ROS 2 Jazzy package that implements a car-like **Hybrid A*** path planner in C++, with a lightweight Python/Tkinter map editor for rapid testing and RViz visualization.

This project is designed for planning feasible paths for nonholonomic vehicles on a 2D occupancy grid. It publishes the final path as a `nav_msgs/Path` and visualizes vehicle footprints along the solution in RViz.

## Highlights

- C++ Hybrid A* planner for car-like motion
- Occupancy-grid collision checking using a rectangular vehicle footprint
- Analytic goal connection using Dubins and Reeds-Shepp curves
- Obstacle-aware heuristic and clearance-based cost shaping
- RViz visualization for the planned path and footprint markers
- Simple Tkinter GUI for drawing obstacles and setting start/goal poses
- Clean separation between planner logic, collision checking, and curve generation

## Current Status

The package is fully usable for **interactive grid-based planning demos** with ROS 2 and RViz.

Current workflow:
- Publish a grid on `astar_grid`
- Set a start pose on `/initialpose`
- Set a goal pose on `/goal_pose`
- Receive the planned path on `astar_path`

Planned next step:
- Gazebo simulation integration with a vehicle model and planner-in-the-loop testing

## Package Contents

- `src/planner_node.cpp` — ROS 2 planner node
- `src/hybrid_astar.cpp` — Hybrid A* search implementation
- `src/grid_collision.cpp` — occupancy-grid collision and distance checks
- `src/curves.cpp` — Dubins / Reeds-Shepp analytic expansions
- `scripts/gui_node.py` — Tkinter map editor and input publisher

## Dependencies

ROS 2 Jazzy packages used by this project:
- `rclcpp`
- `nav_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `tf2`
- `tf2_geometry_msgs`

Frontend utility:
- Python 3
- `rclpy`
- `tkinter`

## Build

```bash
cd ~/ros2_jazzy
colcon build --packages-select hybrid_astar_cpp
source install/setup.bash
