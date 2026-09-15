// Copyright 2026 FZI Forschungszentrum Informatik
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

// Regression test for the deprecated-Handle-API migration: the exported state/command
// interface *names* must exactly match what the old raw-pointer export_state_interfaces()/
// export_command_interfaces() used to export, one-for-one. Nothing here exercises hardware or
// the UR client library - only on_init() + export via the hardware_interface::System wrapper
// (which runs the same old-export-empty -> on_export_state_interfaces() fallback the real
// resource_manager uses). The HardwareInfo built here transcribes ur.ros2_control.xacro's
// joint/sensor/gpio declarations exactly, since every interface this driver exports is now
// URDF-declared (no export_unlisted_*_interface_descriptions() override is needed at all).

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "ur_robot_driver/hardware_interface.hpp"

namespace
{
const std::vector<std::string> kJointNames = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };

hardware_interface::InterfaceInfo make_interface(const std::string& name)
{
  hardware_interface::InterfaceInfo iface{};
  iface.name = name;
  return iface;
}

hardware_interface::ComponentInfo make_gpio(const std::string& name, const std::vector<std::string>& command_names,
                                             const std::vector<std::string>& state_names)
{
  hardware_interface::ComponentInfo gpio;
  gpio.name = name;
  for (const auto& n : command_names) {
    gpio.command_interfaces.push_back(make_interface(n));
  }
  for (const auto& n : state_names) {
    gpio.state_interfaces.push_back(make_interface(n));
  }
  return gpio;
}

std::vector<std::string> numbered(const std::string& prefix, size_t count, size_t start = 0)
{
  std::vector<std::string> names;
  for (size_t i = start; i < start + count; ++i) {
    names.push_back(prefix + std::to_string(i));
  }
  return names;
}

hardware_interface::HardwareInfo build_test_hardware_info()
{
  hardware_interface::HardwareInfo info;
  info.type = "system";
  info.hardware_parameters["tf_prefix"] = "";

  for (const auto& joint_name : kJointNames) {
    hardware_interface::ComponentInfo joint;
    joint.name = joint_name;
    joint.command_interfaces = { make_interface(hardware_interface::HW_IF_POSITION),
                                  make_interface(hardware_interface::HW_IF_VELOCITY),
                                  make_interface(hardware_interface::HW_IF_EFFORT) };
    joint.state_interfaces = { make_interface(hardware_interface::HW_IF_POSITION),
                                make_interface(hardware_interface::HW_IF_VELOCITY),
                                make_interface(hardware_interface::HW_IF_EFFORT) };
    info.joints.push_back(joint);
  }

  hardware_interface::ComponentInfo fts_sensor;
  fts_sensor.name = "tcp_fts_sensor";
  fts_sensor.state_interfaces = { make_interface("force.x"),  make_interface("force.y"),  make_interface("force.z"),
                                   make_interface("torque.x"), make_interface("torque.y"), make_interface("torque.z") };
  info.sensors.push_back(fts_sensor);

  hardware_interface::ComponentInfo tcp_pose_sensor;
  tcp_pose_sensor.name = "tcp_pose";
  tcp_pose_sensor.state_interfaces = { make_interface("position.x"),    make_interface("position.y"),
                                        make_interface("position.z"),    make_interface("orientation.x"),
                                        make_interface("orientation.y"), make_interface("orientation.z"),
                                        make_interface("orientation.w") };
  info.sensors.push_back(tcp_pose_sensor);

  info.gpios.push_back(
      make_gpio("speed_scaling", { "target_speed_fraction_cmd", "target_speed_fraction_async_success" },
                { "speed_scaling_factor" }));

  {
    std::vector<std::string> gpio_cmds = numbered("standard_digital_output_cmd_", 18);
    gpio_cmds.push_back("standard_analog_output_cmd_0");
    gpio_cmds.push_back("standard_analog_output_cmd_1");
    gpio_cmds.push_back("analog_output_domain_cmd");
    gpio_cmds.push_back("tool_voltage_cmd");
    gpio_cmds.push_back("io_async_success");

    std::vector<std::string> gpio_states = numbered("digital_output_", 18);
    for (const auto& n : numbered("digital_input_", 18)) {
      gpio_states.push_back(n);
    }
    gpio_states.push_back("standard_analog_output_0");
    gpio_states.push_back("standard_analog_output_1");
    gpio_states.push_back("standard_analog_input_0");
    gpio_states.push_back("standard_analog_input_1");
    for (const auto& n : numbered("analog_io_type_", 4)) {
      gpio_states.push_back(n);
    }
    gpio_states.push_back("tool_mode");
    gpio_states.push_back("tool_output_voltage");
    gpio_states.push_back("tool_output_current");
    gpio_states.push_back("tool_temperature");
    gpio_states.push_back("tool_analog_input_0");
    gpio_states.push_back("tool_analog_input_1");
    gpio_states.push_back("tool_analog_input_type_0");
    gpio_states.push_back("tool_analog_input_type_1");
    gpio_states.push_back("robot_mode");
    for (const auto& n : numbered("robot_status_bit_", 4)) {
      gpio_states.push_back(n);
    }
    gpio_states.push_back("safety_mode");
    for (const auto& n : numbered("safety_status_bit_", 11)) {
      gpio_states.push_back(n);
    }
    gpio_states.push_back("program_running");
    info.gpios.push_back(make_gpio("gpio", gpio_cmds, gpio_states));
  }

  info.gpios.push_back(make_gpio("payload", { "mass", "cog.x", "cog.y", "cog.z", "payload_async_success" },
                                  { "mass", "cog.x", "cog.y", "cog.z" }));

  {
    std::vector<std::string> friction_cmds = numbered("viscous_", 6);
    for (const auto& n : numbered("coulomb_", 6)) {
      friction_cmds.push_back(n);
    }
    friction_cmds.push_back("async_success");
    info.gpios.push_back(make_gpio("friction_model", friction_cmds, {}));
  }

  info.gpios.push_back(
      make_gpio("resend_robot_program", { "resend_robot_program_cmd", "resend_robot_program_async_success" }, {}));
  info.gpios.push_back(
      make_gpio("hand_back_control", { "hand_back_control_cmd", "hand_back_control_async_success" }, {}));
  info.gpios.push_back(make_gpio("zero_ftsensor", { "zero_ftsensor_cmd", "zero_ftsensor_async_success" }, {}));
  info.gpios.push_back(make_gpio("freedrive_mode", { "async_success", "enable", "abort" }, {}));
  info.gpios.push_back(
      make_gpio("tool_contact", { "tool_contact_set_state" }, { "tool_contact_result", "tool_contact_state" }));
  info.gpios.push_back(make_gpio("system_interface", {}, { "initialized" }));

  {
    const std::vector<std::string> axes = { "x", "y", "z", "rx", "ry", "rz" };
    std::vector<std::string> force_mode_cmds;
    for (const auto& suffix : { std::string("task_frame_"), std::string("selection_vector_"), std::string("wrench_"),
                                 std::string("limits_") }) {
      for (const auto& axis : axes) {
        force_mode_cmds.push_back(suffix + axis);
      }
    }
    force_mode_cmds.push_back("type");
    force_mode_cmds.push_back("damping");
    force_mode_cmds.push_back("gain_scaling");
    force_mode_cmds.push_back("disable_cmd");
    force_mode_cmds.push_back("force_mode_async_success");
    info.gpios.push_back(make_gpio("force_mode", force_mode_cmds, {}));
  }

  {
    std::vector<std::string> passthrough_cmds = numbered("setpoint_positions_", 6);
    for (const auto& n : numbered("setpoint_velocities_", 6)) {
      passthrough_cmds.push_back(n);
    }
    for (const auto& n : numbered("setpoint_accelerations_", 6)) {
      passthrough_cmds.push_back(n);
    }
    passthrough_cmds.push_back("transfer_state");
    passthrough_cmds.push_back("time_from_start");
    passthrough_cmds.push_back("abort");
    passthrough_cmds.push_back("trajectory_size");
    info.gpios.push_back(make_gpio("trajectory_passthrough", passthrough_cmds, {}));
  }

  info.gpios.push_back(make_gpio(
      "get_robot_software_version", {},
      { "get_version_major", "get_version_minor", "get_version_build", "get_version_bugfix" }));

  return info;
}

std::vector<std::string> get_names(const std::vector<hardware_interface::StateInterface::ConstSharedPtr>& interfaces)
{
  std::vector<std::string> names;
  names.reserve(interfaces.size());
  for (const auto& interface : interfaces) {
    names.push_back(interface->get_name());
  }
  return names;
}

std::vector<std::string> get_names(const std::vector<hardware_interface::CommandInterface::SharedPtr>& interfaces)
{
  std::vector<std::string> names;
  names.reserve(interfaces.size());
  for (const auto& interface : interfaces) {
    names.push_back(interface->get_name());
  }
  return names;
}

// Reproduces exactly what the removed raw-pointer export_state_interfaces() used to build.
std::vector<std::string> expected_state_names()
{
  std::vector<std::string> names;
  for (const auto& joint_name : kJointNames) {
    names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  names.push_back("speed_scaling/speed_scaling_factor");
  for (const auto& n : { "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z" }) {
    names.push_back(std::string("tcp_fts_sensor/") + n);
  }
  for (size_t i = 0; i < 18; ++i) {
    names.push_back("gpio/digital_output_" + std::to_string(i));
    names.push_back("gpio/digital_input_" + std::to_string(i));
  }
  for (size_t i = 0; i < 11; ++i) {
    names.push_back("gpio/safety_status_bit_" + std::to_string(i));
  }
  for (size_t i = 0; i < 4; ++i) {
    names.push_back("gpio/analog_io_type_" + std::to_string(i));
    names.push_back("gpio/robot_status_bit_" + std::to_string(i));
  }
  for (size_t i = 0; i < 2; ++i) {
    names.push_back("gpio/tool_analog_input_type_" + std::to_string(i));
    names.push_back("gpio/tool_analog_input_" + std::to_string(i));
    names.push_back("gpio/standard_analog_input_" + std::to_string(i));
    names.push_back("gpio/standard_analog_output_" + std::to_string(i));
  }
  names.push_back("gpio/tool_output_voltage");
  names.push_back("gpio/robot_mode");
  names.push_back("gpio/safety_mode");
  names.push_back("gpio/tool_mode");
  names.push_back("gpio/tool_output_current");
  names.push_back("gpio/tool_temperature");
  names.push_back("system_interface/initialized");
  names.push_back("gpio/program_running");
  names.push_back("tcp_pose/position.x");
  names.push_back("tcp_pose/position.y");
  names.push_back("tcp_pose/position.z");
  names.push_back("tcp_pose/orientation.x");
  names.push_back("tcp_pose/orientation.y");
  names.push_back("tcp_pose/orientation.z");
  names.push_back("tcp_pose/orientation.w");
  names.push_back("get_robot_software_version/get_version_major");
  names.push_back("get_robot_software_version/get_version_minor");
  names.push_back("get_robot_software_version/get_version_bugfix");
  names.push_back("get_robot_software_version/get_version_build");
  names.push_back("tool_contact/tool_contact_result");
  names.push_back("tool_contact/tool_contact_state");
  names.push_back("payload/mass");
  names.push_back("payload/cog.x");
  names.push_back("payload/cog.y");
  names.push_back("payload/cog.z");
  return names;
}

// Reproduces exactly what the removed raw-pointer export_command_interfaces() used to build.
std::vector<std::string> expected_command_names()
{
  std::vector<std::string> names;
  for (const auto& joint_name : kJointNames) {
    names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  names.push_back("gpio/io_async_success");
  names.push_back("speed_scaling/target_speed_fraction_cmd");
  names.push_back("speed_scaling/target_speed_fraction_async_success");
  names.push_back("resend_robot_program/resend_robot_program_cmd");
  names.push_back("resend_robot_program/resend_robot_program_async_success");
  names.push_back("hand_back_control/hand_back_control_cmd");
  names.push_back("hand_back_control/hand_back_control_async_success");
  names.push_back("payload/mass");
  names.push_back("payload/cog.x");
  names.push_back("payload/cog.y");
  names.push_back("payload/cog.z");
  names.push_back("payload/payload_async_success");
  for (size_t i = 0; i < 6; ++i) {
    names.push_back("friction_model/viscous_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i) {
    names.push_back("friction_model/coulomb_" + std::to_string(i));
  }
  names.push_back("friction_model/async_success");
  for (const auto& axis : { "x", "y", "z", "rx", "ry", "rz" }) {
    names.push_back(std::string("force_mode/task_frame_") + axis);
  }
  for (const auto& axis : { "x", "y", "z", "rx", "ry", "rz" }) {
    names.push_back(std::string("force_mode/selection_vector_") + axis);
  }
  for (const auto& axis : { "x", "y", "z", "rx", "ry", "rz" }) {
    names.push_back(std::string("force_mode/wrench_") + axis);
  }
  names.push_back("force_mode/type");
  for (const auto& axis : { "x", "y", "z", "rx", "ry", "rz" }) {
    names.push_back(std::string("force_mode/limits_") + axis);
  }
  names.push_back("force_mode/force_mode_async_success");
  names.push_back("force_mode/disable_cmd");
  names.push_back("force_mode/damping");
  names.push_back("force_mode/gain_scaling");
  for (size_t i = 0; i < 18; ++i) {
    names.push_back("gpio/standard_digital_output_cmd_" + std::to_string(i));
  }
  for (size_t i = 0; i < 2; ++i) {
    names.push_back("gpio/standard_analog_output_cmd_" + std::to_string(i));
  }
  names.push_back("gpio/analog_output_domain_cmd");
  names.push_back("gpio/tool_voltage_cmd");
  names.push_back("zero_ftsensor/zero_ftsensor_cmd");
  names.push_back("zero_ftsensor/zero_ftsensor_async_success");
  names.push_back("freedrive_mode/async_success");
  names.push_back("freedrive_mode/enable");
  names.push_back("freedrive_mode/abort");
  names.push_back("trajectory_passthrough/transfer_state");
  names.push_back("trajectory_passthrough/time_from_start");
  names.push_back("trajectory_passthrough/abort");
  names.push_back("trajectory_passthrough/trajectory_size");
  for (size_t i = 0; i < 6; ++i) {
    names.push_back("trajectory_passthrough/setpoint_positions_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i) {
    names.push_back("trajectory_passthrough/setpoint_velocities_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i) {
    names.push_back("trajectory_passthrough/setpoint_accelerations_" + std::to_string(i));
  }
  names.push_back("tool_contact/tool_contact_set_state");
  return names;
}

class URPositionHardwareInterfaceExportTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = build_test_hardware_info();
    params.clock = std::make_shared<rclcpp::Clock>();
    params.logger = rclcpp::get_logger("test_ur_robot_driver_hardware_interface_exports");

    hw_ = std::make_unique<hardware_interface::System>(
        std::make_unique<ur_robot_driver::URPositionHardwareInterface>());
    const auto state = hw_->initialize(params);
    ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "on_init() did not succeed - check the test HardwareInfo matches what "
           "URPositionHardwareInterface::on_init() validates.";
  }

  std::unique_ptr<hardware_interface::System> hw_;
};

TEST_F(URPositionHardwareInterfaceExportTest, ExportsExpectedStateInterfaces)
{
  const auto exported_names = get_names(hw_->export_state_interfaces());
  EXPECT_THAT(exported_names, ::testing::UnorderedElementsAreArray(expected_state_names()));
}

TEST_F(URPositionHardwareInterfaceExportTest, ExportsExpectedCommandInterfaces)
{
  const auto exported_names = get_names(hw_->export_command_interfaces());
  EXPECT_THAT(exported_names, ::testing::UnorderedElementsAreArray(expected_command_names()));
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
