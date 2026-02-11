# ROS_Gazebo_based_simulations
 This is a an attempt to perform Ros2- Gazebo based simulations to test algorithms like A* and GVD for path generation and following the bot over a custom environment / world
 
# ROS2 Dual-Planner Navigation System  
## Comparative Implementation of A* and Generalized Voronoi Diagram (GVD)

---

## 1. Introduction

This project presents a complete dual-planner navigation framework built in ROS2 (Humble), designed to compare two fundamentally different path-planning philosophies:

- **A*** — Optimal shortest-path planning
- **GVD (Generalized Voronoi Diagram)** — Safety-oriented medial axis planning

The objective of this work was to investigate the trade-off between **optimality and safety** in autonomous mobile robot navigation and to implement both planners from scratch within a unified ROS2 simulation environment.

The system was tested in Gazebo using a TurtleBot3 model with occupancy grid maps served via `nav2_map_server`.

---

## 2. Motivation

In autonomous navigation, shortest path planning is often not the safest strategy in cluttered environments. While A* guarantees optimality in path length, it may pass dangerously close to obstacles.

In contrast, GVD generates paths that maximize clearance by following the medial axis of free space. Although longer in distance, these paths improve robustness in narrow corridors and constrained environments.

This project was designed to:

- Implement both planners independently
- Build appropriate path-following controllers for each
- Compare behavioral differences in simulation
- Understand controller-planner coupling effects

---

## 3. System Architecture

The system follows a modular ROS2 architecture:

Map Server → Planner Server → Path Topic → Path Follower → /cmd_vel → Gazebo Robot


### Core Components:

- `planner_server.py`  
  Implements both A* and GVD planning logic.

- `path_follower.py`  
  Lookahead-based controller for A*.

- `path_follower_gvd.py`  
  Sequential waypoint controller for GVD.

- `gauntlet_nav.launch.py`  
  Launch orchestration and planner selection.

---

## 4. Planner Implementations

### 4.1 A* Planner

The A* algorithm was implemented manually using:

- 8-connected grid expansion
- Euclidean heuristic
- Priority queue (`heapq`)
- Manual `g` and `f` cost tracking
- Path reconstruction via parent mapping

Key properties:

- Guarantees shortest path
- Efficient node expansion
- Sharp turns in narrow regions
- May approach obstacles closely

---

### 4.2 GVD Planner

The GVD planner generates a medial-axis skeleton of the free space.

Core characteristics:

- Maximizes obstacle clearance
- Produces safer but longer trajectories
- Especially effective in corridor-like environments
- Generates higher curvature paths

The implementation focuses on extracting safe centerlines from the occupancy grid and publishing them as a `nav_msgs/Path`.

---

## 5. Path Following Strategies

Two different controllers were designed to match planner behavior.

### A* Controller

- Lookahead-based tracking
- Continuous heading correction
- Smooth motion prioritizing efficiency

### GVD Controller

- Sequential waypoint tracking
- Rotate-then-drive behavior
- Guarantees faithful traversal of medial axis
- Prioritizes correctness over smoothness

This distinction highlights that controller design must align with planner characteristics.

---

## 6. Experimental Observations

### A* Behavior

- Shortest travel distance
- Aggressive turns
- Reduced travel time
- Lower obstacle clearance

### GVD Behavior

- Longer trajectory
- Higher obstacle clearance
- Safer corridor traversal
- Clear medial-axis alignment

These experiments demonstrate the fundamental trade-off:

> Shortest path ≠ Safest path

---

## 7. Engineering Challenges Solved

During development, several system-level issues were resolved:

- ROS2 QoS mismatch between Gazebo and subscribers
- Lifecycle-managed Map Server configuration
- Repeated path publishing causing follower reset loops
- Controller oscillations at high-curvature turns
- Synchronization of planner execution timing

Each issue required debugging across multiple ROS2 subsystems including lifecycle management, QoS policies, and node orchestration.

---

## 8. Technologies Used

- ROS2 Humble (rclpy)
- Gazebo Simulation
- nav2_map_server
- RViz2
- TurtleBot3
- Python
- Occupancy Grid Mapping

---

## 9. How to Run

### Build the workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Launch with A*
```
ros2 launch gauntlet_sim gauntlet_nav.launch.py planner_type:=astar
```
Launch with GVD
```
ros2 launch gauntlet_sim gauntlet_nav.launch.py planner_type:=gvd
```
10. Key Learnings

    * Optimality and safety are competing objectives in navigation.

    * Controller strategy must be tailored to planner characteristics.

    * Medial-axis planning improves robustness in constrained spaces.

    * ROS2 lifecycle and QoS configuration are critical for stable system behavior.

    * Debugging distributed robotic systems requires cross-layer understanding.

11. Future Improvements

    * Add RRT or sampling-based planner comparison

    * Implement dynamic obstacle avoidance

    * Add curvature-aware smoothing

    * Quantitatively measure clearance vs. distance trade-offs

    * Integrate costmaps and dynamic replanning

12. Author

Ayush Bhaskar
Mechanical Engineering
IIT Bombay

Areas of Interest:

    Autonomous Systems

    UAV Navigation

    Robotics Control

    Intelligent Mobility Systems
