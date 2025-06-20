// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Eigen/Dense>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <franka_example_controllers/robot_utils.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"


#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include "geometry_msgs/msg/twist.hpp"
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_srvs/srv/trigger.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * joint impedance example controller get desired pose and use inverse kinematics LMA
 * (Levenberg-Marquardt) from Orocos KDL. IK returns the desired joint positions from the desired
 * pose. Desired joint positions are fed to the impedance control law together with the current
 * joint velocities to calculate the desired joint torques.
 */
class JointImpedanceWithIKExampleController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  void omegaButtonCallback(std_msgs::msg::Bool::SharedPtr msg);
  void FdEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void FdEETwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void netFTCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void homeButtonCallback(std_msgs::msg::Bool::SharedPtr msg);
  void replayJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

 private:
  void update_joint_states();
  void init_services();
  void onFirstReplayJointState(const std::vector<double>& joints) ;
  void replayReadyCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  geometry_msgs::msg::PoseStamped fd_ee_pose_;
  geometry_msgs::msg::Twist fd_ee_twist_;
  
  Eigen::Vector3d com_{-0.0, -0.0, -0.0};
  double mass_ = 0.87;
  double gravity_ = 9.81;

  geometry_msgs::msg::WrenchStamped netft_raw_;
  geometry_msgs::msg::WrenchStamped netft_comp_;
  Eigen::Vector3d pos_org_;
  
  Eigen::Quaterniond ori_org_;

  Eigen::Vector3d prev_target_position_;
  
  Eigen::Vector3d post_org_;
  Eigen::Quaterniond orit_org_;
  Eigen::Vector3d angt_org_;
  bool is_gripper_loaded_ = true;

  double button_init_ = 1;
  bool button_check_{false};

  std::vector<double> replay_joint_positions_;
  std::atomic<bool> has_replay_msg_{false};
  
  std::vector<double> first_replay_joint_positions_;

  bool move_to_first_replay_pose_{false};
  std::atomic<bool> reached_first_position_{false};

  double replay_pose_tolerance_ = 0.02; // radians, adjust as needed
  rclcpp::Time replay_move_start_time_;
  double replay_move_duration_ = 2.0; // seconds, adjust as needed
  std::vector<double> replay_start_positions_;

  rclcpp::Time last_replay_msg_time_;

  /**
   * @brief Calculates the new pose based on the initial pose.
   *
   * @return  Eigen::Vector3d calculated sinosuidal period for the x,z position of the pose.
   */
  Eigen::Vector3d compute_new_position();

  /**
   * @brief creates the ik service request for ik service from moveit. Assigns the move-group,
   * desired pose of the desired link.
   *
   * @return std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> request service message
   */
  std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> create_ik_service_request(
      const Eigen::Vector3d& new_position,
      const Eigen::Quaterniond& new_orientation,
      const std::vector<double>& joint_positions_desired,
      const std::vector<double>& joint_positions_current,
      const std::vector<double>& joint_efforts_current);

  /**
   * @brief computes the torque commands based on impedance control law with compensated coriolis
   * terms
   *
   * @return Eigen::Vector7d torque for each joint of the robot
   */
  Vector7d compute_torque_command(const Vector7d& joint_positions_desired,
                                  const Vector7d& joint_positions_current,
                                  const Vector7d& joint_velocities_current);

  /**
   * @brief assigns the Kp, Kd and arm_id parameters
   *
   * @return true when parameters are present, false when parameters are not available
   */
  bool assign_parameters();

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;

  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  std::string arm_id_;
  std::string robot_description_;
  
  double trajectory_period_{0.001};

  double elapsed_time_{0.0};
  double initial_robot_time_{0.0};
  double robot_time_{0.0};
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  int num_joints_{7};

  std::vector<double> joint_positions_desired_;
  std::vector<double> joint_positions_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_velocities_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_efforts_current_{0, 0, 0, 0, 0, 0, 0};

  double dt_{0.001};
  double cutoff_freq_ = 1;
  double ramp_up_duration_{0.01};
  double pos_scale_{0.5};
  double rot_scale_{0.3};

  // 홈 모드 관련
  bool   home_button_check_{false};
  int    home_button_init_{0};
  rclcpp::Time   home_start_time_;
  std::array<double,7> home_start_positions_;
  std::array<double,7> home_positions_;
  double home_move_duration_{2.0};  // 예: 5초에 걸쳐 이동

  rclcpp::Time t_ramp_start_;
  /// 전역 또는 클래스 멤버 변수
  Eigen::Vector3d cumulative_offset = Eigen::Vector3d::Zero();
  Eigen::Vector3d cumulative_orientation_offset = Eigen::Vector3d::Zero();
  // ramp up이 진행 중일 때 쓰일 현재 ramp ratio
  double ramp_ratio_{0.0};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr omegaButton_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_button_sub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fd_ee_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr fd_ee_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr netft_sub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr replay_jointstate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr replay_first_jointstate_sub_;

  //! Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_poset_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr netft_comp_pub_;


  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replay_ready_service_;





};
}  // namespace franka_example_controllers