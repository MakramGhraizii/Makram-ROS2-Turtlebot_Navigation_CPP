# Makram-ROS2-Turtlebot_Navigation_C++

This project provides a ROS 2 package for controlling and navigating a TurtleBot using C++. The package includes nodes for driving the robot, finding walls, and measuring lap times.

### Prerequisites

Ensure you have the following installed:

- ROS 2 (Foxy, Galactic, Humble, or later)
- colcon
- a C++ compiler (e.g., gcc)

### Setting up the Workspace

Create and set up your ROS 2 workspace:

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## Building the Package

1. Clone the repository into the `src` directory:

    ```sh
    cd ~/ros2_ws/src
    git clone <repository_url> turtlebot_navigation_cpp
    ```

2. Source the ROS 2 setup script:

    ```sh
    source /opt/ros/<ros2_distro>/setup.bash
    ```

    Replace `<ros2_distro>` with your ROS 2 distribution name (e.g., `foxy`, `galactic`, `humble`).

3. Build the package using `colcon`:

    ```sh
    cd ~/ros2_ws
    colcon build
    ```

4. Source the workspace setup script:

    ```sh
    source install/setup.bash
    ```

## Running the Nodes

### robot_driver

Launch the `robot_driver` node:

```sh
ros2 run turtlebot_navigation_cpp robot_driver
```

### wall_finder

Launch the `wall_finder` node:

```sh
ros2 run turtlebot_navigation_cpp wall_finder
```

### lap_time_action_server

Launch the `lap_time_action_server` node:

```sh
ros2 run turtlebot_navigation_cpp lap_time_action_server
```

### lap_time_action_client

Launch the `lap_time_action_client` node:

```sh
ros2 run turtlebot_navigation_cpp lap_time_action_client
```

## Nodes Overview

### robot_driver

The `robot_driver` node is responsible for controlling the movement of the TurtleBot. It subscribes to relevant topics and sends commands to the robot's motors.

### wall_finder

The `wall_finder` node uses sensor data to detect walls and obstacles in the environment. It publishes information about the detected walls to a topic.

### lap_time_action_server

The `lap_time_action_server` node implements an action server that tracks the time taken to complete a lap. It provides feedback and result messages to action clients.

### lap_time_action_client

The `lap_time_action_client` node acts as a client for the lap time action server. It sends goals to the server and processes the responses.

## Launch File

The package includes a launch file to start multiple nodes together, with GAZEBO as well You can run the launch file with the following command:

```sh
ros2 launch turtlebot_navigation_cpp launch.py
```
