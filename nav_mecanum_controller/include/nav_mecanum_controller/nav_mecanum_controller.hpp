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

#ifndef NAV_MECANUM_CONTROLLER__NAV_MECANUM_CONTROLLER_HPP_
#define NAV_MECANUM_CONTROLLER__NAV_MECANUM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "nav_mecanum_controller_parameters.hpp"
#include "nav_mecanum_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace nav_mecanum_controller
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};

class NavMecanumController : public controller_interface::ControllerInterface
{
public:
  NAV_MECANUM_CONTROLLER__VISIBILITY_PUBLIC
  NavMecanumController();

  NAV_MECANUM_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  NAV_MECANUM_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  NAV_MECANUM_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  NAV_MECANUM_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  NAV_MECANUM_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  NAV_MECANUM_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  NAV_MECANUM_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerStateMsg = geometry_msgs::msg::Twist;
  using OdomMsg = nav_msgs::msg::Odometry;


protected:
  std::shared_ptr<nav_mecanum_controller::ParamListener> param_listener_;
  nav_mecanum_controller::Params params_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<OdomMsg>> odom_input_ref_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  // callback for nav_msgs::msg::Odometry topic interface
  NAV_MECANUM_CONTROLLER__VISIBILITY_LOCAL
  void odom_callback(const std::shared_ptr<OdomMsg> msg);

  NAV_MECANUM_CONTROLLER__VISIBILITY_LOCAL
  void reset_controller_reference_msg(std::shared_ptr<ControllerReferenceMsg>);
};

}  // namespace nav_mecanum_controller

#endif  // NAV_MECANUM_CONTROLLER__NAV_MECANUM_CONTROLLER_HPP_
