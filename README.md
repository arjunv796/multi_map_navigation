Multi-Map Navigation for ROS 2
Seamlessly Guiding Robots Across Disconnected Worlds

Navigating complex, multi-level environments like office buildings or warehouses is a significant challenge for autonomous robots. Standard navigation systems like Nav2 excel at moving within a single, continuous map but struggle when a robot must transition between disconnected spaces (e.g., different floors via an elevator).

This project offers an elegant, high-level solution: a multi-map coordinator that intelligently manages navigation goals across these separate maps. It acts as a "brain" on top of Nav2, handling the complex logic of map transitions so the rest of the system doesn't have to.
The Core Concept: The Wormhole System

At the heart of this project is the "wormhole" concept. Think of wormholes as virtual doorways or elevators that connect your maps.

    A SQLite database stores the precise coordinates of these connection points on each map.

    When the robot is on "Map A" and receives a goal for "Map B," the server's logic is simple:

        Look up the wormhole connecting Map A to Map B.

        Navigate the robot to the wormhole's location on Map A.

        Once it arrives, trigger the map-switching process (e.g., call an elevator, notify the map server to load Map B).

        Proceed to the final goal on Map B.

This approach cleanly separates the high-level task planning (which map to be on) from the low-level navigation (how to move on the current map).
Key Features

    Strategic Goal Coordination: Implements a robust ROS 2 action server (/navigate_to_map) that serves as the primary entry point for all long-range navigation tasks.

    Intelligent Map Transitioning: Automatically plans and executes the necessary steps to move a robot between maps using the wormhole system.

    Persistent Data Storage: Leverages a lightweight SQLite database to maintain a persistent and easily editable record of map connection points.

    Composable & Efficient: Designed as a ROS 2 component, allowing it to be run as a standalone node or loaded into a component container for optimized, zero-copy communication.

    Ready to Deploy: Includes a pre-configured Python launch file for straightforward startup and parameter configuration.

Getting Started
Prerequisites

    ROS 2 Humble Hawksbill

    The Nav2 (Navigation2) Stack

    SQLite3 Development Library (sudo apt-get install libsqlite3-dev)

Installation

    Clone the Repository:
    Clone this project into your ROS 2 workspace's src directory.

    cd ~/your_ros2_workspace/src
    git clone <your-repository-url>

    Install Dependencies:
    Navigate to your workspace root and let rosdep handle dependency installation.

    cd ~/your_ros2_workspace
    rosdep install --from-paths src --ignore-src -r -y

    Build the Package:
    Compile the workspace with colcon.

    cd ~/your_ros2_workspace
    colcon build

How to Use
1. Source Your Workspace

In every new terminal, source your workspace to make its packages available.

source ~/your_ros2_workspace/install/setup.bash

2. Launch the Server

Use the provided launch file to start the multi_map_server. This command automatically finds and passes the path to your wormhole.db to the node.

ros2 launch multi_map_navigation multi_map_server.launch.py

You'll see log output confirming the node is running and the action server is active and waiting for goals.
3. Send a Test Goal

From another terminal, you can send a test goal using the CLI. This command simulates an application requesting the robot to move to position (1.0, 1.0) on map1.

ros2 action send_goal /navigate_to_map multi_map_navigation/action/NavigateToMap '{
  "map_id": "map1",
  "goal_pose": {
    "header": { "frame_id": "map" },
    "pose": {
      "position": { "x": 1.0, "y": 1.0, "z": 0.0 },
      "orientation": { "w": 1.0 }
    }
  }
}'

Observe the server's terminal to see how it receives and processes the request.
Project Roadmap

This project provides a solid foundation for multi-map navigation. Future enhancements could include:

    Full Nav2 Integration: Directly connecting the server's output to Nav2's NavigateToPose action client for seamless, end-to-end execution.

    Dynamic Wormhole Management: Implementing a ROS 2 service to allow for adding, removing, or updating wormhole locations in the database on-the-fly.

    Advanced Map Transition Logic: Developing more sophisticated plugins for different transition types, such as automatic door opening or elevator control.
