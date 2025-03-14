// Copyright 2023 ros2_control Development Team
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

#include "kr_robot_driver/kr810_hardware.hpp"
#include <string>
#include <vector>

using namespace kr2;

namespace kr_robot_driver
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // robot has 7 joints and 2 interfaces
  joint_position_.assign(7, 0);
  joint_velocities_.assign(7, 0);
  joint_position_command_.assign(7, 0);
  joint_velocities_command_.assign(7, 0);

  // force sensor has 6 readings
  ft_states_.assign(6, 0);
  ft_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }
  std::shared_ptr<kord::KordCore> kord(new kord::KordCore("192.168.39.1", 7582, 1, kord::UDP_CLIENT));
  kord::ControlInterface ctl_iface(kord);
    kord::ReceiverInterface rcv_iface(kord);
  // Establish connection to the robot's CBun.
  if (!kord->connect()) {
    std::cout << "Connecting to KR failed\n";
    return CallbackReturn::ERROR;
  }

  // Send ArmStatus request to initiate control session.
  if (!kord->syncRC()){
    std::cout << "Sync RC failed.\n";
    return CallbackReturn::ERROR;
  }
  if (!kord->waitSync(std::chrono::milliseconds(10), 1)){
    std::cout << "Sync RC failed.\n";
    return CallbackReturn::ERROR;
    // return EXIT_FAILURE;
}
  std::cout << "Sync Captured \n";
  rcv_iface.fetchData();
  unsigned int digital_output = rcv_iface.getDigitalOutput();
  std::cout << rcv_iface.getFormattedOutputBits() << std::endl;
  std::array<double, 7UL> start_q = rcv_iface.getJoint(kord::ReceiverInterface::EJointValue::S_ACTUAL_Q);
  std::cout << "Sync Captured \n";
  std::cout <<"joints values: "<< start_q[0] << std::endl;
  std::cout << "Sync Captured \n";

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  // Mock interface from the demo, actual values are identical to commanded

  for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  {
    joint_velocities_[i] = joint_velocities_command_[i];
    joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  }

  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    joint_position_[i] = joint_position_command_[i];
  }

  // Reading the state from the KORD API

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

}  // namespace kr_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kr_robot_driver::RobotSystem, hardware_interface::SystemInterface)
