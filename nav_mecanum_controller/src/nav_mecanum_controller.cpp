// Copyright (c) 2023, pansamic
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include "nav_mecanum_controller/nav_mecanum_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

  using JointMsg = sensor_msgs::msg::JointState;
  using IMUMsg = sensor_msgs::msg::Imu;

// called from RT control loop
void reset_imu_msg(std::shared_ptr<IMUMsg> & msg)
{
  msg->header.stamp = rclcpp::Time(0);
  msg->header.frame_id = "";
  msg->orientation.x = std::numeric_limits<double>::quiet_NaN();
  msg->orientation.y = std::numeric_limits<double>::quiet_NaN();
  msg->orientation.z = std::numeric_limits<double>::quiet_NaN();
  msg->orientation.w = std::numeric_limits<double>::quiet_NaN();
  msg->angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
  msg->angular_velocity.y = std::numeric_limits<double>::quiet_NaN();
  msg->angular_velocity.z = std::numeric_limits<double>::quiet_NaN();
  msg->linear_acceleration.x = std::numeric_limits<double>::quiet_NaN();
  msg->linear_acceleration.y = std::numeric_limits<double>::quiet_NaN();
  msg->linear_acceleration.z = std::numeric_limits<double>::quiet_NaN();
  msg->orientation_covariance = {0,0,0,0,0,0,0,0,0};
  msg->angular_velocity_covariance = {0,0,0,0,0,0,0,0,0};
  msg->linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};
}

void reset_joint_state_msg(std::shared_ptr<JointMsg> & msg, std::vector<string> & joint_names)
{
  msg->header.stamp = rclcpp::Time(0);
  msg->name = joint_names;
  msg->position.resize(joint_names.size());
  msg->position = std::vector<double>(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocity.resize(joint_names.size());
  msg->velocity = std::vector<double>(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->effort.resize(joint_names.size());
  msg->effort = std::vector<double>(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
}

}  // namespace

namespace nav_mecanum_controller
{
NavMecanumController::NavMecanumController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn NavMecanumController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<nav_mecanum_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NavMecanumController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  odom_subscriber_ = get_node()->create_subscription<OdomMsg>(
    params_.odom_subscribe_topic, subscribers_qos,
    std::bind(&NavMecanumController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  odom_input_ref_.writeFromNonRT(msg);

  try
  {
    // State publisher
    s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(params_.twist_publish_topic, rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.joints[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void NavMecanumController::odom_callback(const std::shared_ptr<OdomMsg> msg)
{
  odom_odom_input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration NavMecanumController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration NavMecanumController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn NavMecanumController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for examplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  // Set default value in command
  reset_controller_reference_msg(*(odom_input_ref_.readFromRT)(), params_.joints);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NavMecanumController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type NavMecanumController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref = odom_input_ref_.readFromRT();

  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (!std::isnan((*current_ref)->displacements[i]))
    {
      if (*(control_mode_.readFromRT()) == control_mode_type::SLOW)
      {
        (*current_ref)->displacements[i] /= 2;
      }
      command_interfaces_[i].set_value((*current_ref)->displacements[i]);

      (*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace nav_mecanum_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nav_mecanum_controller::NavMecanumController, controller_interface::ControllerInterface)
