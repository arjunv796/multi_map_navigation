Multi-Map Navigation for ROS 2 — Seamlessly Guiding Robots Across Disconnected Worlds

1. Overview
Navigating complex, multi-level environments like office buildings or warehouses is a significant challenge for autonomous robots.
Standard navigation systems like Nav2 excel at moving within a single, continuous map but struggle when a robot must transition between disconnected spaces (e.g., different floors via an elevator).

This project offers a high-level multi-map coordinator that intelligently manages navigation goals across separate maps. It acts as a “brain” on top of Nav2, handling complex logic for map transitions so the rest of the system doesn’t have to.

2. Core Concept — The Wormhole System
At the heart of this project is the "wormhole" concept — virtual doorways or elevators that connect your maps.

1. A SQLite database stores the precise coordinates of these connection points on each map.
2. When the robot is on Map A and receives a goal for Map B, the logic is:
   1. Look up the wormhole connecting Map A to Map B.
   2. Navigate the robot to the wormhole's location on Map A.
   3. Once it arrives, trigger the map-switching process (e.g., call an elevator, load Map B in the map server).
   4. Proceed to the final goal on Map B.

Benefit: This approach cleanly separates high-level task planning (which map to be on) from low-level navigation (how to move within a map).

3. Key Features

1. Strategic Goal Coordination
   - Implements a robust ROS 2 Action Server (/navigate_to_map) as the entry point for all long-range navigation tasks.

2. Intelligent Map Transitioning
   - Automatically plans and executes steps to move between maps using the wormhole system.

3. Persistent Data Storage
   - Lightweight SQLite database maintains persistent, editable records of wormhole coordinates.

4. Composable & Efficient
   - Designed as a ROS 2 component node; can run standalone or in a component container for zero-copy communication.

5. Ready to Deploy
   - Includes a pre-configured Python launch file for quick startup and parameter configuration.

4. Getting Started

4.1 Prerequisites
1. ROS 2 Humble Hawksbill
2. Nav2 (Navigation2) Stack
3. SQLite3 Development Library
   sudo apt-get install libsqlite3-dev

4.2 Installation

1. Clone the Repository
   cd ~/your_ros2_workspace/src
   git clone <your-repository-url>

2. Install Dependencies
   cd ~/your_ros2_workspace
   rosdep install --from-paths src --ignore-src -r -y

3. Build the Package
   cd ~/your_ros2_workspace
   colcon build

4.3 How to Use

1. Source Your Workspace
   source ~/your_ros2_workspace/install/setup.bash

2. Launch the Server
   ros2 launch multi_map_navigation multi_map_server.launch.py
   - This will start the multi_map_server node and load your wormhole.db file.

3. Send a Test Goal
   ros2 action send_goal /navigate_to_map multi_map_navigation/action/NavigateToMap    '{ "map_id": "map1", "goal_pose": { "header": { "frame_id": "map" }, "pose": { "position": { "x": 1.0, "y": 1.0, "z": 0.0 }, "orientation": { "w": 1.0 } } } }'

5. Project Roadmap

1. Full Nav2 Integration
   - Directly connect action server output to Nav2’s NavigateToPose for seamless execution.

2. Dynamic Wormhole Management
   - Implement a ROS 2 service to add, remove, or update wormhole locations in real time.

3. Advanced Map Transition Logic
   - Develop plugins for different transitions, such as automatic doors or elevator control.
