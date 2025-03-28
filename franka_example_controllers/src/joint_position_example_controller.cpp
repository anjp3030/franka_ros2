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
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <franka_example_controllers/joint_position_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace std::chrono_literals;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
<<<<<<< HEAD
JointPositionExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }

  // add the robot time interface
  if (!is_gazebo_) {
    config.names.push_back(arm_id_ + "/robot_time");
  }

  return config;
=======
  JointPositionExampleController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    }
    return config;
  }
  
  controller_interface::InterfaceConfiguration
  JointPositionExampleController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = franka_cartesian_pose_->get_state_interface_names();
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    }
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    }
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
    // for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    //   config.names.push_back(franka_robot_model_name);
    // }
  
    return config;
  }
  void JointPositionExampleController::update_joint_states() {
    for (auto i = 0; i < num_joints_; ++i) {
      // TODO(yazi_ba) Can we get the state from its name?
      const auto& position_interface = state_interfaces_.at(16 + i);
      const auto& velocity_interface = state_interfaces_.at(23 + i);
      const auto& effort_interface = state_interfaces_.at(30 + i);
      joint_positions_current_[i] = position_interface.get_value();
      joint_velocities_current_[i] = velocity_interface.get_value();
      joint_efforts_current_[i] = effort_interface.get_value();
    }
  }
Eigen::Quaterniond JointPositionExampleController::RotationToQuaternion(const Eigen::Quaterniond& current_orientation,
  const Eigen::Vector3d& current_angle) {
  // 1. 오일러 각을 사용해 회전 쿼터니언 생성 (ZYX 순서로 적용: Yaw -> Pitch -> Roll)
  Eigen::Quaterniond roll_quaternion(Eigen::AngleAxisd(current_angle.x(), Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond pitch_quaternion(Eigen::AngleAxisd(current_angle.y(), Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond yaw_quaternion(Eigen::AngleAxisd(current_angle.z(), Eigen::Vector3d::UnitX()));
  Eigen::Quaterniond compensate_quaternion(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  // 2. 오일러 각 회전 쿼터니언들을 곱해 최종 회전 쿼터니언 계산 (순서: Yaw -> Pitch -> Roll)
  Eigen::Quaterniond euler_rotation = yaw_quaternion * pitch_quaternion * roll_quaternion;

  // 3. 기존 쿼터니언에 새로운 회전 쿼터니언을 곱해 추가 회전 적용
  Eigen::Quaterniond new_orientation = current_orientation * euler_rotation;
  // Eigen::Vector3d euler_angles = new_orientation.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX 순서
  // std::cout << "Yaw (Z): " << roll * 180.0 / M_PI << "° "
  //       << "Pitch (Y): " << pitch * 180.0 / M_PI << "° "
  //       << "Roll (X): " << yaw * 180.0 / M_PI << "° " << std::endl;
  // 4. 정규화 후 반환
  return new_orientation.normalized();
}

std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request>
JointPositionExampleController::create_ik_service_request(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::vector<double>& joint_positions_current,
    const std::vector<double>& joint_velocities_current,
    const std::vector<double>& joint_efforts_current) {
  auto service_request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

  service_request->ik_request.group_name = arm_id_ + "_arm";
  service_request->ik_request.pose_stamped.header.frame_id = arm_id_ + "_link0";
  service_request->ik_request.pose_stamped.pose.position.x = position.x();
  service_request->ik_request.pose_stamped.pose.position.y = position.y();
  service_request->ik_request.pose_stamped.pose.position.z = position.z();
  service_request->ik_request.pose_stamped.pose.orientation.x = orientation.x();
  service_request->ik_request.pose_stamped.pose.orientation.y = orientation.y();
  service_request->ik_request.pose_stamped.pose.orientation.z = orientation.z();
  service_request->ik_request.pose_stamped.pose.orientation.w = orientation.w();
  service_request->ik_request.robot_state.joint_state.name = {
      arm_id_ + "_joint1", arm_id_ + "_joint2", arm_id_ + "_joint3", arm_id_ + "_joint4",
      arm_id_ + "_joint5", arm_id_ + "_joint6", arm_id_ + "_joint7"};
  service_request->ik_request.robot_state.joint_state.position = joint_positions_current;
  service_request->ik_request.robot_state.joint_state.velocity = joint_velocities_current;
  service_request->ik_request.robot_state.joint_state.effort = joint_efforts_current;

  // If Franka Hand is not connected, the following line should be commented out.
  service_request->ik_request.ik_link_name = arm_id_ + "_hand_tcp";
  return service_request;
}
void JointPositionExampleController::omegaButtonCallback(const std_msgs::msg::Bool::SharedPtr msg){
  button_check_ = msg->data;
}


void JointPositionExampleController::FdEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
this->fd_ee_pose_ = *msg;
}
void JointPositionExampleController::FdEETwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
  this->fd_ee_twist_ = *msg; 
}

void JointPositionExampleController::netFTCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {

netft_raw_ = *msg;

Eigen::Matrix3d R_base_ee = orientation_.toRotationMatrix();;
// std::cout<<R_base_ee * R_base_ee.transpose()<<std::endl;
Eigen::Vector3d g_base(0.0, 0.0, -gravity_);

// Eigen::Matrix3d R_z = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()).toRotationMatrix();

Eigen::Vector3d g_ee = R_base_ee.transpose() * g_base;
Eigen::Vector3d Fgo = mass_ * g_base;
Eigen::Vector3d Tgo = com_.cross(Fgo);
Eigen::Vector3d Fg = mass_ * g_ee;
Eigen::Vector3d Tg = com_.cross(Fg);
// 2) 중력 보상 계산
//  (a) 측정된 힘/토크
double Fx = msg->wrench.force.x;
double Fy = msg->wrench.force.y;
double Fz = msg->wrench.force.z;
double Tx = msg->wrench.torque.x;
double Ty = msg->wrench.torque.y;
double Tz = msg->wrench.torque.z;



Eigen::Vector3d F_meas(Fy, -Fx, -Fz);
Eigen::Vector3d T_meas(Ty, -Tx, -Tz);


Eigen::Vector3d F_comp = F_meas + Fg + Fgo;
Eigen::Vector3d T_comp = T_meas + Tg + Tgo;
// std::cout<<"R :"<< R_base_ee <<std::endl;
// std::cout<<"Fg :"<< Fg <<std::endl;
// std::cout<<"Fgo :"<< Fgo <<std::endl;




// 6. 보정된 Wrench 메시지 생성
netft_comp_ = *msg;  // 헤더 그대로 복사
netft_comp_.wrench.force.x  = (-F_comp.x());
netft_comp_.wrench.force.y  = (F_comp.y());
netft_comp_.wrench.force.z  = (F_comp.z());
netft_comp_.wrench.torque.x = -T_comp.x();
netft_comp_.wrench.torque.y = T_comp.y();
netft_comp_.wrench.torque.z = T_comp.z();

// 7. 보정된 값 퍼블리시
netft_comp_pub_->publish(netft_comp_);
>>>>>>> 16219e2 (add Teleoperation for omega7)
}

controller_interface::return_type JointPositionExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
    if (initialization_flag_) {
      std::tie(orientation_, position_) =
          franka_cartesian_pose_->getInitialOrientationAndTranslation();
      initialization_flag_ = false;
      pos_org_ = position_;
      ori_org_ = orientation_;
    }
<<<<<<< HEAD
    initialization_flag_ = false;
    if (!is_gazebo_) {
      initial_robot_time_ = state_interfaces_.back().get_value();
    }
    elapsed_time_ = 0.0;
  } else {
    if (!is_gazebo_) {
      robot_time_ = state_interfaces_.back().get_value();
      elapsed_time_ = robot_time_ - initial_robot_time_;
    } else {
      elapsed_time_ += trajectory_period_;
    }
  }

  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_)) * 0.2;
=======

    update_joint_states();
>>>>>>> 16219e2 (add Teleoperation for omega7)

    Eigen::Vector3d final_position = position_;
    Eigen::Quaterniond final_orientation = orientation_;
  
    Eigen::Vector3d target_position(
      -fd_ee_pose_.pose.position.x,
      -fd_ee_pose_.pose.position.y,
      fd_ee_pose_.pose.position.z
    );
    Eigen::Vector3d target_angle(
      -fd_ee_twist_.angular.z,
      fd_ee_twist_.angular.y,
      -fd_ee_twist_.angular.x
    );
  
  
    // -------------------------------
    // teleoperation 시작 시 (버튼 누름 전환 시 초기화)
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
      Eigen::Vector3d diff_position = target_position - post_org_;
      Eigen::Vector3d new_position_desired = pos_org_ + diff_position;
      final_position = new_position_desired;
      
      // 누적 위치 오프셋 갱신
      cumulative_offset = diff_position;
      
      // 회전 업데이트
      // 현재 teleoperation 입력과 초기 기준(orit_org_) 사이의 상대 회전 계산
      Eigen::Vector3d angle_diff = target_angle - angt_org_;
  
      
      // 새 목표 회전: 초기 기준 회전에 상대 회전을 적용
      Eigen::Quaterniond new_orientation_desired = RotationToQuaternion(ori_org_ , angle_diff);
      new_orientation_desired.normalize();  // 정규화 권장
      final_orientation = new_orientation_desired;
      
      // 누적 회전 오프셋 갱신 (q_diff를 누적)
      cumulative_orientation_offset = angle_diff;
      
      // (매 사이클 기준 쿼터니언(orit_org_)을 갱신하고 싶다면 아래 주석 해제)
      // orit_org_ = target_orientation;
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
    }
  
  
  
    auto service_request =
        create_ik_service_request(final_position, final_orientation, joint_positions_current_,
                                  joint_velocities_current_, joint_efforts_current_);
  
    using ServiceResponseFuture = rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture;
    auto response_received_callback =
        [&](ServiceResponseFuture future) {  // NOLINT(performance-unnecessary-value-param)
          const auto& response = future.get();
  
          if (response->error_code.val == response->error_code.SUCCESS) {
            joint_positions_desired_ = response->solution.joint_state.position;
          } else {
            RCLCPP_INFO(get_node()->get_logger(), "Inverse kinematics solution failed.");
          }
        };
    auto result_future_ =
        compute_ik_client_->async_send_request(service_request, response_received_callback);
  
    if (joint_positions_desired_.empty()) {
      return controller_interface::return_type::OK;
    }
  
    Vector7d joint_positions_desired_eigen(joint_positions_desired_.data());
    Vector7d joint_positions_current_eigen(joint_positions_current_.data());
    Vector7d joint_velocities_current_eigen(joint_velocities_current_.data());
  
    
    for (int i = 0; i < num_joints_; i++) {
      command_interfaces_[i].set_value(joint_positions_current_eigen(i));
    }
  
    return controller_interface::return_type::OK;
  }
  
  CallbackReturn JointPositionExampleController::on_init() {
    franka_cartesian_pose_ =
        std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
            franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));
  
    return CallbackReturn::SUCCESS;
  }
  
CallbackReturn JointPositionExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    // franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
    //     franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
    //                                                  arm_id_ + "/" + k_robot_state_interface_name));
  
    auto collision_client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "/service_server/set_full_collision_behavior");
    compute_ik_client_ = get_node()->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
  
    while (!compute_ik_client_->wait_for_service(1s) || !collision_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return CallbackReturn::ERROR;
      }
      RCLCPP_INFO(get_node()->get_logger(), "service not available, waiting again...");
    }
  
    fd_ee_pose_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "fd/ee_pose", rclcpp::SystemDefaultsQoS(),
        std::bind(&JointPositionExampleController::FdEEPoseCallback, this, std::placeholders::_1));
    
    fd_ee_twist_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "fd/ee_twist", rclcpp::SystemDefaultsQoS(),
        std::bind(&JointPositionExampleController::FdEETwistCallback, this, std::placeholders::_1));
    // 1) netft 데이터 구독
    netft_sub_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "netft_data",  // 실제 netft 데이터 토픽 이름
      rclcpp::SystemDefaultsQoS(),
      std::bind(&JointPositionExampleController::netFTCallback, this, std::placeholders::_1));
  
    omegaButton_sub_ =
      get_node()->create_subscription<std_msgs::msg::Bool>(
        "fd/button_state", rclcpp::SystemDefaultsQoS(),
        std::bind(&JointPositionExampleController::omegaButtonCallback, this, std::placeholders::_1));
    
    ee_pose_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "ee_pose", rclcpp::SystemDefaultsQoS());
    ee_poset_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "ee_poset", rclcpp::SystemDefaultsQoS());
            
  
  
  
    // 2) 보상된 Wrench 퍼블리셔
    netft_comp_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "netft_data_compensated",
            rclcpp::SystemDefaultsQoS());
  
    button_check_ = false;
    prev_target_position_ = Eigen::Vector3d::Zero();
  
    auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
    auto future_result = collision_client->async_send_request(request);
  
    auto success = future_result.get();
    arm_id_ = get_node()->get_parameter("arm_id").as_string();

  
    if (!success->success) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
      return CallbackReturn::ERROR;
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
    }
  
    return CallbackReturn::SUCCESS;
  }

CallbackReturn JointPositionExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;

  joint_positions_desired_.reserve(num_joints_);
  joint_positions_current_.reserve(num_joints_);
  joint_velocities_current_.reserve(num_joints_);
  joint_efforts_current_.reserve(num_joints_);

  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  // franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn JointPositionExampleController::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
franka_cartesian_pose_->release_interfaces();
return CallbackReturn::SUCCESS;
}


}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerInterface)
