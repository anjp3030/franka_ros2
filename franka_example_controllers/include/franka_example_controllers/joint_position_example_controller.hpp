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

#include <string>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
// #include "franka_semantic_components/franka_robot_state.hpp"

#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"
#include <moveit_msgs/srv/get_position_ik.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include "geometry_msgs/msg/twist.hpp"
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The joint position example controller moves in a periodic movement.
 */
class JointPositionExampleController : public controller_interface::ControllerInterface {
 public:
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

 private:

  void update_joint_states();
  Eigen::Quaterniond RotationToQuaternion(const Eigen::Quaterniond& current_orientation,
    const Eigen::Vector3d& current_angle); 
  geometry_msgs::msg::PoseStamped fd_ee_pose_;
  geometry_msgs::msg::Twist fd_ee_twist_;
  
  Eigen::Vector3d com_{-0.0, -0.0, -0.0};
  double mass_ = 1.071;
  double gravity_ = 9.81;

  geometry_msgs::msg::WrenchStamped netft_raw_;
  geometry_msgs::msg::WrenchStamped netft_comp_;
  Eigen::Vector3d pos_org_;
  
  Eigen::Quaterniond ori_org_;

  Eigen::Vector3d prev_target_position_;
  
  Eigen::Vector3d post_org_;
  Eigen::Quaterniond orit_org_;
  Eigen::Vector3d angt_org_;

  double button_init_ = 1;

  bool button_check_{false};

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


  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;

  double trajectory_period_{0.001};
  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  std::string arm_id_;

  double elapsed_time_{0.0};
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  int num_joints_{7};

  std::vector<double> joint_positions_desired_;
  std::vector<double> joint_positions_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_velocities_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_efforts_current_{0, 0, 0, 0, 0, 0, 0};

  bool is_gazebo_{false};
  std::string robot_description_;

  std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0};
  double initial_robot_time_ = 0.0;
  double robot_time_ = 0.0;
  rclcpp::Time start_time_;


  /// 전역 또는 클래스 멤버 변수
  Eigen::Vector3d cumulative_offset = Eigen::Vector3d::Zero();
  Eigen::Vector3d cumulative_orientation_offset = Eigen::Vector3d::Zero();
  // ramp up이 진행 중일 때 쓰일 현재 ramp ratio
  double ramp_ratio_{0.0};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr omegaButton_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fd_ee_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr fd_ee_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr netft_sub_;


  //! Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_poset_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr netft_comp_pub_;
};

}  // namespace franka_example_controllers
