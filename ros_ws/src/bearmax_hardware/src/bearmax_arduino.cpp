#include "bearmax_hardware/bearmax_arduino.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bearmax_hardware
{
hardware_interface::CallbackReturn BearmaxArduino::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Initializing...");

    // Get parameters
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);

    hw_servo_states_.resize(
        info_.joints.size(), std::numeric_limits<int>::quiet_NaN());

    hw_servo_cmds_.resize(
        info_.joints.size(), std::numeric_limits<int>::quiet_NaN());

    // Setup Arduino
    arduino_.setup(cfg_.device, cfg._baud_rate, cfg.timeout);

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Finished Initialization");

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> BearmaxArduino::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;


    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_servo_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BearmaxArduino::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_servo_cmds_[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn BearmaxArduino::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{

    // Set default values.
    for (auto i = 0u; i < hw_servo_states_.size(); i++) {
        if (std::isnan(hw_servo_states_[i])) {
            hw_servo_states_[i] = 0;
            hw_servo_cmds_[i] = 0;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearmaxArduino::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{

    // TODO: Close BLE connection

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BearmaxArduino::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!arduino_.connected()) {
        return hardware_interface::return_type::ERROR;
    }

    // TODO: Read servo data into hw_servo_states_

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type BearmaxArduino::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!arduino_.connected()) {
        return hardware_interface::return_type::ERROR;
    }

    arduino_.setServoValues(hw_servo_cmds_);

    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    bearmax_hardware::BearmaxArduino,
    hardware_interface::SystemInterface)

