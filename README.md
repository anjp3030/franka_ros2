# JointImpedanceWithIKExampleController

[franka_example_controllers](https://github.com/frankaemika/franka_ros2) 패키지를 기반으로 하며, 7자유도(7-DOF) 로봇(예: Franka Panda)에서 텔레오퍼레이션, 동작 재생, 힘 보상 작업 등에 사용할 수 있습니다.
```
franka_ros2/franka_example_controllers/src/joint_impedance_with_ik_example_controller.cpp
````
## 주요 기능

- **조인트 임피던스 제어**: 사용자 지정 강성(`k_gains`) 및 감쇠(`d_gains`) 설정값을 사용해 조인트 공간에서 임피던스 제어를 수행합니다.
```
franka_ros2/franka_bringup/config/controllers.yaml
```
다음 파일에서 각 조인트에 대한 k_gain 과 d_gain을 수정할 수 
```
joint_impedance_with_ik_example_controller:
  ros__parameters:
    k_gains:
      - 600.0
      - 600.0
      - 600.0
      - 600.0
      - 250.0
      - 150.0
      - 50.0
    d_gains:
      - 30.0
      - 30.0
      - 30.0
      - 30.0
      - 10.0
      - 10.0
      - 5.
```
- **IK Solver**: MoveIt!의 `/compute_ik` 서비스를 이용해 목표 카티시안 포즈를 조인트 각도로 변환합니다.변환된 조인트 각도는 다음 함수를 통해 토크로 변환됩니다.
```
Vector7d JointImpedanceWithIKExampleController::compute_torque_command(
    const Vector7d& joint_positions_desired,
    const Vector7d& joint_positions_current,
    const Vector7d& joint_velocities_current) {
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Vector7d coriolis(coriolis_array.data());

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current;
  Vector7d q_error = joint_positions_desired - joint_positions_current;
  Vector7d tau_d_calculated = 
      k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_) + coriolis;

  return tau_d_calculated;
}

```
- **Teleoperation**: 실시간 텔레오퍼레이션을 위해 햅틱(Omega 7) 장치에서 받은 포즈 및 트위스트 명령을 받아 처리합니다.
  Omega 7을 버튼으로 입력을 받아 입력을 받은 시점의 위치를 기억한 후 입력을 받은 시점에서부터 위치에 Remote로봇의 위치뱌를 업데이트 받아 움직입니다.()

  ```
  `fd/ee_pose` (`geometry_msgs/PoseStamped`): 원하는 엔드 이펙터(EE) 포즈 (주로 텔레오퍼레이션에서 사용)
  `fd/ee_twist` (`geometry_msgs/Twist`): 원하는 엔드 이펙터 트위스트
  ```
  ```
    if (button_check_ && button_init_ == 0) {
    // 위치: 이전 누적 오프셋 반영 및 기준점 설정
    pos_org_ = pos_org_ + cumulative_offset;
    post_org_ = target_position;  // 초기 기준점 설정
    
    // 회전: 이전 누적 회전 오프셋 반영 후, 초기 기준 쿼터니언을 현재 target_orientation으로 설정
    ori_org_ = RotationToQuaternion(ori_org_ , cumulative_orientation_offset);
    angt_org_ = target_angle;  // 초기 기준 쿼터니언 설정 (target_angle에서 변환된 값)
    
    button_init_ = 1;
  }

  // -------------------------------
  // teleoperation 활성 상태에서, 누적 오프셋 업데이트
  if (button_check_) {
    // 위치 업데이트
    Eigen::Vector3d diff_position = (target_position - post_org_) * pos_scale_;
    Eigen::Vector3d new_position_desired = pos_org_ + diff_position ;
    final_position = new_position_desired;
    
    // 누적 위치 오프셋 갱신
    cumulative_offset = diff_position;
    
    // 회전 업데이트
    // 현재 teleoperation 입력과 초기 기준(orit_org_) 사이의 상대 회전 계산
    Eigen::Vector3d angle_diff = (target_angle - angt_org_) * rot_scale_;

    
    // 새 목표 회전: 초기 기준 회전에 상대 회전을 적용
    Eigen::Quaterniond new_orientation_desired = RotationToQuaternion(ori_org_ , angle_diff);
    new_orientation_desired.normalize();  // 정규화 권장
    final_orientation = new_orientation_desired;
    orientation_ = final_orientation;

    // 누적 회전 오프셋 갱신 (q_diff를 누적)
    cumulative_orientation_offset = angle_diff;

    
  } else {
    // teleoperation 종료 시, 누적 오프셋 최종 반영
    if (button_init_ == 1) {
      pos_org_ = pos_org_ + cumulative_offset;
      cumulative_offset.setZero();
      
      ori_org_ = RotationToQuaternion(ori_org_ , cumulative_orientation_offset);
      ori_org_.normalize();
      
      cumulative_orientation_offset.setZero();
      
      button_init_ = 0;
    }
    final_position = pos_org_;
    final_orientation = ori_org_;
    orientation_ = final_orientation;

  }

  ```

- **힘-토크 보상**: NetFT 센서로부터 데이터를 구독하고, 중력 및 질량 보상을 적용하여 보정된 렌치(wrench) 데이터를 퍼블리시합니다.
```
'netFTCallback' 다음 함수 참고 
```
- **주요 홈 및 재생 모드**: "Home" 버튼 지원 및 구독한 토픽을 통해 조인트 궤적 재생이 가능합니다.
```
`home_button` (`std_msgs/Bool`)
```
홈 모드 버튼을 누르면 홈으로 움직입니다.
```
`replay_first_jointstate` (`sensor_msgs/JointState`)
```
 replay를 하기 위해 첫 위치로 움직인 후 service를 응답하여 Gui에서 나머지 위치를 구독하게 합니다. 

```
`replay_jointstate` (`sensor_msgs/JointState`)
```
나머지 jointstate를 받아 replay를 실행합니다. 

## 구독하는 토픽

- `fd/ee_pose` (`geometry_msgs/PoseStamped`): 원하는 엔드 이펙터(EE) 포즈 (주로 텔레오퍼레이션에서 사용)
- `fd/ee_twist` (`geometry_msgs/Twist`): 원하는 엔드 이펙터 트위스트
- `fd/button_state` (`std_msgs/Bool`): 텔레오퍼레이션 버튼 입력
- `home_button` (`std_msgs/Bool`): 홈 모드 버튼
- `replay_jointstate` (`sensor_msgs/JointState`): 재생 모드용 조인트 위치
- `replay_first_jointstate` (`sensor_msgs/JointState`): 재생 모드의 첫 위치
- `netft_data` (`geometry_msgs/WrenchStamped`): 힘-토크 센서 데이터

## 발행하는 토픽

- `ee_pose` (`geometry_msgs/PoseStamped`): 현재 엔드 이펙터 포즈
- `ee_poset` (`geometry_msgs/PoseStamped`): (옵션) 또 다른 포즈 퍼블리셔
- `netft_data_compensated` (`geometry_msgs/WrenchStamped`): 중력 및 질량 보정이 적용된 힘-토크 데이터

## 서비스

- `replay_ready` (`std_srvs/Trigger`): 로봇이 재생 모드의 첫 번째 자세에 도달했는지 여부를 알려줌


## 의존성

- ROS 2 
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
