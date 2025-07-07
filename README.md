# JointImpedanceWithIKExampleController

This ROS 2 controller implements joint impedance control with inverse kinematics (IK) for Franka Emika robots.  
It is based on the [franka_example_controllers](https://github.com/frankaemika/franka_ros2) package and can be used for teleoperation, replay, and force-compensated tasks with 7-DOF robots (e.g., Franka Panda).

## Features

- **Joint Impedance Control**: Applies impedance control in joint space using user-configurable stiffness (`k_gains`) and damping (`d_gains`).
- **Inverse Kinematics Service**: Uses the MoveIt! `/compute_ik` service to convert Cartesian target pose to joint angles.
- **Teleoperation**: Receives pose and twist commands (e.g., from a haptic device or joystick) for real-time teleop.
- **Home and Replay Modes**: Supports "Home" button and joint trajectory replay via subscribed topics.
- **Force-Torque Compensation**: Subscribes to a NetFT sensor, applies gravity and mass compensation, and publishes compensated wrench data.
- **Service Integration**: Service to notify when the robot reaches the first replay pose (`replay_ready`).
- **Plug-and-Play with Franka ROS 2 and MoveIt**: Designed for easy integration with existing Franka and MoveIt setups.

## Subscribed Topics

- `fd/ee_pose` (`geometry_msgs/PoseStamped`): Desired end-effector pose (usually from teleoperation).
- `fd/ee_twist` (`geometry_msgs/Twist`): Desired end-effector twist.
- `fd/button_state` (`std_msgs/Bool`): Teleoperation button input.
- `home_button` (`std_msgs/Bool`): Home mode button.
- `replay_jointstate` (`sensor_msgs/JointState`): Joint positions for replay mode.
- `replay_first_jointstate` (`sensor_msgs/JointState`): First position for replay.
- `netft_data` (`geometry_msgs/WrenchStamped`): Force-torque sensor data.

## Published Topics

- `ee_pose` (`geometry_msgs/PoseStamped`): Current end-effector pose.
- `ee_poset` (`geometry_msgs/PoseStamped`): (Optional) Another pose publisher.
- `netft_data_compensated` (`geometry_msgs/WrenchStamped`): Gravity- and mass-compensated force-torque data.

## Services

- `replay_ready` (`std_srvs/Trigger`): Reports whether the robot reached the first replay pose.

## Parameters

- `arm_id` (string): Name prefix for the Franka robot, e.g., `panda`.
- `load_gripper` (bool): Whether a gripper is loaded for IK calculation.
- `k_gains` (double array): Joint stiffness gains, length = 7.
- `d_gains` (double array): Joint damping gains, length = 7.

## Dependencies

- ROS 2 (tested on Humble and Rolling)
- [franka_ros2](https://github.com/frankaemika/franka_ros2)
- [moveit_ros2](https://moveit.ros.org/)
- [geometry_msgs](https://github.com/ros2/common_interfaces/tree/humble/geometry_msgs)
- [std_msgs](https://github.com/ros2/common_interfaces/tree/humble/std_msgs)
- [sensor_msgs](https://github.com/ros2/common_interfaces/tree/humble/sensor_msgs)
- [std_srvs](https://github.com/ros2/common_interfaces/tree/humble/std_srvs)



<h1 style="font-size: 3em;">ROS 2 Integration for Franka Robotics Research Robots</h1>

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

> **Note:** _franka_ros2_ is not officially supported on Windows.

#### Table of Contents
- [About](#about)
- [Caution](#caution)
- [Setup](#setup)
  - [Local Machine Installation](#local-machine-installation)
  - [Docker Container Installation](#docker-container-installation)
- [Test the Setup](#test-the-setup)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

# About
The **franka_ros2** repository provides a **ROS 2** integration of **libfranka**, allowing efficient control of the Franka Robotics arm within the ROS 2 framework. This project is designed to facilitate robotic research and development by providing a robust interface for controlling the research versions of Franka Robotics robots.

For convenience, we provide Dockerfile and docker-compose.yml files. While it is possible to build **franka_ros2** directly on your local machine, this approach requires manual installation of certain dependencies, while many others will be automatically installed by the **ROS 2** build system (e.g., via **rosdep**). This can result in a large number of libraries being installed on your system, potentially causing conflicts. Using Docker encapsulates these dependencies within the container, minimizing such risks. Docker also ensures a consistent and reproducible build environment across systems. For these reasons, we recommend using Docker.

# Caution
This package is in rapid development. Users should expect breaking changes and are encouraged to report any bugs via [GitHub Issues page](https://github.com/frankaemika/franka_ros2/issues).

# Franka ROS 2 Dependencies Setup

This repository contains a `.repos` file that helps you clone the required dependencies for Franka ROS 2.

## Prerequisites

## Local Machine Installation
1. **Install ROS2 Development environment**

    _**franka_ros2**_ is built upon _**ROS 2 Humble**_.  

    To set up your ROS 2 environment, follow the official _**humble**_ installation instructions provided [**here**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). 
    The guide discusses two main installation options: **Desktop** and **Bare Bones**.

    #### Choose **one** of the following:
    - **ROS 2 "Desktop Install"** (`ros-humble-desktop`)  
      Includes a full ROS 2 installation with GUI tools and visualization packages (e.g., Rviz and Gazebo).  
      **Recommended** for users who need simulation or visualization capabilities.

    - **"ROS-Base Install (Bare Bones)"** (`ros-humble-ros-base`)  
      A minimal installation that includes only the core ROS 2 libraries.  
      Suitable for resource-constrained environments or headless systems.

    ```bash
    # replace <YOUR CHOICE> with either ros-humble-desktop or ros-humble-ros-base
    sudo apt install <YOUR CHOICE>  
    ```
    ---
    Also install the **Development Tools** package:
    ```bash
    sudo apt install ros-dev-tools
    ```
    Installing the **Desktop** or **Bare Bones** should automatically source the **ROS2** environment but, under some circumstances you may need to do this again:
    ```bash
    source /opt/ros/humble/setup.sh
    ```

2. **Create a ROS 2 Workspace:**
   ```bash
   mkdir -p ~/franka_ros2_ws/src
   cd ~/franka_ros2_ws  # not into src
   ```
3. **Clone the Repositories:**
   ```bash
    git clone https://github.com/frankaemika/franka_ros2.git src
    ```
4. **Install the dependencies**
    ```bash
    vcs import src < src/franka.repos --recursive --skip-existing
    ```
5. **Detect and install project dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```
6. **Build**
   ```bash
   # use the --symlinks option to reduce disk usage, and facilitate development.
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
7. **Adjust Enviroment**
   ```bash
   # Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
   source install/setup.sh
   ```

## Docker Container Installation
The **franka_ros2** package includes a `Dockerfile` and a `docker-compose.yml`, which allows you to use `franka_ros2` packages without manually installing **ROS 2**. Also, the support for Dev Containers in Visual Studio Code is provided.

For detailed instructions, on preparing VSCode to use the `.devcontainer` follow the setup guide from [VSCode devcontainer_setup](https://code.visualstudio.com/docs/devcontainers/tutorial).

1. **Clone the Repositories:**
    ```bash
    git clone https://github.com/frankaemika/franka_ros2.git
    cd franka_ros2
    ```
    We provide separate instructions for using Docker with Visual Studio Code or the command line. Choose one of the following options:

    Option A: Set up and use Docker from the command line (without Visual Studio Code).

    Option B: Set up and use Docker with Visual Studio Code's Docker support.

#### Option A: using Docker Compose

  2. **Save the current user id into a file:**
      ```bash
      echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
      ```
      It is needed to mount the folder from inside the Docker container.

  3. **Build the container:**
      ```bash
      docker compose build
      ```
  4. **Run the container:**
      ```bash
      docker compose up -d
      ```
  5. **Open a shell inside the container:**
      ```bash
      docker exec -it franka_ros2 /bin/bash
      ```
  6. **Clone the latests dependencies:**
      ```bash
      vcs import src < src/franka.repos --recursive --skip-existing
      ```
  7. **Build the workspace:**
      ```bash
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```
  7. **Source the built workspace:**
      ```bash
      source install/setup.bash
      ```
  8. **When you are done, you can exit the shell and delete the container**:
      ```bash
      docker compose down -t 0
      ```

#### Option B: using Dev Containers in Visual Studio Code

  2. **Open Visual Studio Code ...**
  
        Then, open folder  `franka_ros2`

  3. **Choose `Reopen in container` when prompted.**

      The container will be built automatically, as required.

  4. **Clone the latests dependencies:**
      ```bash
      vcs import src < src/franka.repos --recursive --skip-existing
      ```

  5. **Open a terminal and build the workspace:**
      ```bash
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```
  6. **Source the built workspace environment:**
      ```bash
      source install/setup.bash
      ```


# Test the build
   ```bash
   colcon test
   ```
> Remember, franka_ros2 is under development.  
> Warnings can be expected.  

# Run a sample ROS2 application

To verify that your setup works correctly without a robot, you can run the following command to use dummy hardware:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```


# Troubleshooting
#### `libfranka: UDP receive: Timeout error`

If you encounter a UDP receive timeout error while communicating with the robot, avoid using Docker Desktop. It may not provide the necessary real-time capabilities required for reliable communication with the robot. Instead, using Docker Engine is sufficient for this purpose.

A real-time kernel is essential to ensure proper communication and to prevent timeout issues. For guidance on setting up a real-time kernel, please refer to the [Franka installation documentation](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel).

# Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](https://github.com/frankaemika/franka_ros2/blob/humble/CONTRIBUTING.md) for more details on how to contribute to this project.

## License

All packages of franka_ros2 are licensed under the Apache 2.0 license.

## Contact 

For questions or support, please open an issue on the [GitHub Issues](https://github.com/frankaemika/franka_ros2/issues) page.

See the [Franka Control Interface (FCI) documentation](https://frankaemika.github.io/docs) for more information.


[def]: #docker-container-installation
