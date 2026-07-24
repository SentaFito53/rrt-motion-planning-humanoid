# RRT Motion Planning Test (BarelangFC)

## Objective

This repository/module is a **mini proof-of-concept project** for testing the implementation of the **RRT (Rapidly-exploring Random Tree)** algorithm for motion planning, integrated into the **BarelangFC** humanoid soccer robot control framework (based on ROS2 and BehaviorTree.CPP).

Its primary purpose is **not** production deployment, but rather to:

- Evaluate the feasibility of the RRT algorithm in generating obstacle-free paths within a robot soccer field environment.
- Verify the integration of the path planning algorithm with the existing robot motion control (motion primitives) and odometry system.
- Serve as a foundation for further experiments before applying similar algorithms to real-world tasks, such as avoiding opponent or teammate robots during matches.

---

## Component Structure

| File | Role |
|---|---|
| `rrt_planner.hpp` | Pure implementation of the RRT algorithm (independent of ROS) |
| `main.cpp` (node `main_strategy`) | Integrates the RRT planner into the robot Behavior Tree through the `testGrid` node |
| `rrt_monitor.py` | Separate ROS2 node for visualizing the generated path, obstacles, and robot position on the map |

---

## `NodeStatus testGrid()` Function

`testGrid()` is a **Behavior Tree** node that acts as the state machine controlling the RRT motion planning cycle, consisting of three states (`PlanState`):

1. **`IDLE`** — Executed once at the beginning. It defines the start point `(0,0)`, the goal point `(300,0)`, and a set of manually specified local obstacles (defined by `x`, `y`, and `radius` in centimeters). It then calls `RRTPlanner::plan()` to generate a path, followed by `smoothPath()` to simplify the resulting path. If a valid path is found, the state transitions to `EXECUTING`; otherwise, the node returns `NodeStatus::FAILURE`.

2. **`EXECUTING`** — Executes the generated path one waypoint at a time using the `new_out_pos_norotate()` motion primitive (a closed-loop walk-to-point controller based on odometry feedback without forcing the robot to align its yaw at every waypoint). Once a waypoint is reached (`doneMoved == true`), the waypoint index is incremented, and the robot proceeds to the next waypoint.

3. **`DONE`** — Executed after all waypoints have been reached. The robot is stopped (`motion("0")`), and the planning state and obstacle list are reset to their initial conditions (`IDLE`, `localObstacles.clear()`), allowing the node to be tested again from scratch. The node then returns `NodeStatus::SUCCESS`.

During both the `IDLE` and `EXECUTING` states, the node continuously returns `NodeStatus::FAILURE` so that the Behavior Tree treats it as still running until it finally reaches the `DONE` state. At every tick, `publishRRTVisualization()` is called to publish the latest path and obstacle information to ROS2 topics, enabling real-time monitoring of the planning process.

---

## Implemented RRT Algorithm

The implementation in `rrt_planner.hpp` is a **basic RRT with goal bias**, consisting of the following components:

- **`RRTPlanner::plan(start, goal, obstacles)`** — The main planning function. It grows the exploration tree from the start point using random sampling, with a 10% probability of directly sampling the goal to accelerate convergence. The planner then performs *steering* toward the sampled point with a maximum distance of `stepSize` (20 cm). Each newly generated segment is checked for obstacle collisions using `segmentCircleIntersect()` before being added to the tree. The algorithm terminates once a node is sufficiently close to the goal (`goalTolerance` = 15 cm) and the final segment to the goal is collision-free.

- **`smoothPath(path, obstacles)`** — An optional post-processing function that removes unnecessary intermediate waypoints whenever a direct connection between two waypoints does not intersect any obstacle, resulting in a shorter and smoother path with fewer zig-zag movements.

- **Obstacles** are represented as simple circles `{x, y, radius}`, where the radius includes both the physical size of the opponent robot and an additional safety margin.

---

## Integration with the ROS2 Framework

```text
main_strategy (C++, BehaviorTree.CPP node)
    │
    ├─ testGrid() [state machine IDLE → EXECUTING → DONE]
    │     ├─ RRTPlanner::plan()      → output: vector<Point2D> rrtPath
    │     └─ new_out_pos_norotate()  → closed-loop walk-to-point for each waypoint
    │
    ├─ publish topic 'rrt_path'       (nav_msgs/Path)
    ├─ publish topic 'rrt_obstacles'  (std_msgs/Float32MultiArray, flat format [x,y,r,...])
    └─ publish topic 'pose'           (nav_msgs/Odometry, from the robot odometry system)
              │
              ▼
rrt_monitor.py (Python, separate node)
    ├─ subscribe to 'pose', 'rrt_path', and 'rrt_obstacles'
    └─ render the field map, path, obstacles, and robot position using OpenCV
```

- **`rrt_planner.hpp`** is written in standard C++ with no ROS dependencies, allowing the planning algorithm to be developed and tested independently of the robot control framework.
- **`main_strategy`** bridges the planning output with the existing robot motion control system by using odometry feedback (`robotPos_X/Y`, `msg_yaw`) for closed-loop position control.
- **`rrt_monitor`** runs as a separate ROS2 node (and may even be executed on another computer within the same ROS2 network), providing visual debugging without increasing the computational load of the robot's main strategy node.
- The robot may operate under a namespace (e.g., `/robot_<id>/...`), and `rrt_monitor` supports the `robot_id` parameter to select which robot should be monitored.

---

## 🎥 Demo Video

Watch the RRT motion planning demonstration on YouTube:

[![RRT Motion Planning Demo](https://img.youtube.com/vi/UpEtRjmQ9GM/maxresdefault.jpg)](https://youtu.be/UpEtRjmQ9GM)

> Click the image above to watch the full demonstration.
## Project Status

This mini project is currently **experimental/testing**. Obstacles are manually defined (hardcoded) inside `testGrid()`. Future development may include automatically obtaining obstacle positions from the vision system or inter-robot coordinate sharing, as well as evaluating the planner in more complex game scenarios with dynamic obstacles and realistic field conditions.
