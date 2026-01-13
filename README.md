#Lightweight ROS 2 Navigation & SLAM Simulation

This project provides a lightweight, purely kinematic robot simulation built for ROS 2 Humble. It is specifically optimized for Apple Silicon (M1/M2/M3/M4) architectures and low-resource environments where heavy physics engines like Gazebo are unnecessary or performance-prohibitive.

The simulation utilizes a "Fake Robot" node that generates synthetic Lidar data via ray-casting mathematics, broadcasts TF transforms, and integrates seamlessly with SLAM Toolbox for mapping and Nav2 for autonomous navigation.

##Features

Kinematic Simulation: Eliminates physics engine overhead by using direct mathematical models for movement and sensor generation.

Apple Silicon Native: Runs efficiently inside Docker on ARM64 architectures without emulation.

Full Navigation Stack: Compatible with standard ROS 2 slam_toolbox and nav2_bringup packages.

Integrated Safety: Includes a Watchdog timer to automatically stop the robot if control commands are interrupted.

Data Logging: Capable of recording robot trajectory (Pose and Orientation) to CSV format for analysis.

Visualization: Pre-configured for visualization and control via Foxglove Studio.

##Prerequisites

Docker Desktop: Ensure Docker is installed and running.

Foxglove Studio: Recommended for visualization (Desktop or Web).

Installation

##Clone the repository:

git clone [https://github.com/tojifushiguroed/Robot-Simulation.git](https://github.com/tojifushiguroed/Robot-Simulation.git)
cd Robot-Simulation


Build and start the Docker container:

docker-compose up --build -d


##Workflow Guide

The system is designed to run in multiple terminal tabs, each handling a specific component of the robotic stack.

###1. Base System Setup (Required for All Modes)

Open two separate terminal tabs to start the simulation core and the visualization bridge.

Terminal 1: Robot Simulation Node
This python script acts as the robot hardware interface, generating odometry and lidar data.

docker exec -it ros2_sim_container python3 /root/maps/simple_sim.py


Terminal 2: Foxglove Bridge
This establishes the WebSocket connection for visualization.

docker exec -it ros2_sim_container ros2 launch foxglove_bridge foxglove_bridge_launch.xml


###2. Mode A: Mapping (SLAM)

Use this mode to explore an unknown environment and generate a map.

Terminal 3: SLAM Toolbox

docker exec -it ros2_sim_container ros2 launch slam_toolbox online_async_launch.py


####Operation:

Open Foxglove Studio.

Use the Teleop panel to drive the robot manually.

The map will be generated in real-time on the 3D panel.

####Saving the Map:
Once the area is fully mapped, run the following command in a new terminal window (keep the simulation running):

docker exec -it ros2_sim_container ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/root/maps/my_map'}}"


The map files (my_map.pgm and my_map.yaml) will be saved directly to your local project folder via the Docker volume mount.

###3. Mode B: Autonomous Navigation (Nav2)

Use this mode to make the robot navigate autonomously within a saved map.

Terminal 3: Nav2 Bringup
(Ensure SLAM Toolbox is stopped before running this)

docker exec -it ros2_sim_container ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:=/root/maps/my_map.yaml


####Operation:

Localization: In Foxglove, use "2D Pose Estimate" to indicate the robot's initial position on the map.

Navigation: Use "2D Goal Pose" to set a destination. The robot will plan a path and drive autonomously.

###4. Data Logging

To record the robot's position and orientation over time, run the logger script in a separate terminal.

docker exec -it ros2_sim_container python3 /root/maps/data_logger.py


The data will be saved as a CSV file (e.g., log_20260113_XXXX.csv) in your project directory.

####Foxglove Configuration

To visualize the simulation correctly, configure Foxglove Studio with the following settings:

####Connection:

Type: Foxglove WebSocket

URL: ws://localhost:8765

3D Panel Settings:

Display Frame: map (Essential for correct map visualization)

####Topics:

/map: Set Color Scheme to "Costmap".

/scan: Set Point Size to 3.

/plan: Enables visualization of the global path.

/goal_pose: Visualization of the target destination.

/visualization_marker: Visualizes the robot body (Cyan box).

####Teleop Panel:

Topic: /cmd_vel

Troubleshooting

"Live connection maximum frame size reached" Error

Cause: High data throughput overloading the browser client.

Solution: Decrease the simulation update rate in simple_sim.py (e.g., change timer from 0.1s to 0.2s) and refresh Foxglove (Cmd+R).

Robot stops moving unexpectedly

Cause: The Safety Watchdog timer is triggering due to command latency.

Solution: Increase the watchdog tolerance in simple_sim.py or ensure the network connection is stable.

Ghost Obstacles (Black spots in empty space)

Cause: Network lag causing slight desynchronization between Lidar data and Robot TF during rotation.

Solution: These are temporary artifacts in the local costmap. Drive the robot over the area to clear them via ray-tracing updates.

Robot does not appear on the map (Nav2 Mode)

Cause: The navigation stack does not know the robot's initial position.

Solution: You must perform a "2D Pose Estimate" in Foxglove to initialize the AMCL localization.

##Project Structure

simple_sim.py: Core simulation logic, kinematics, and Lidar generation.

data_logger.py: Utility for logging pose data.

Dockerfile: Environment definition including ROS 2 Humble, Nav2, and SLAM Toolbox.

docker-compose.yaml: Container orchestration configuration.
