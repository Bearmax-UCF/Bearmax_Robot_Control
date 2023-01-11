#include "bearmax_hardware/wearable_sensor_package.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bearmax_hardware
{
hardware_interface::CallbackReturn WearableSensorPackageHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (
        hardware_interface::SensorInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Get parameters
    hw_device_name_ = info_.hardware_parameters["device_name"];

    hw_sensor_states_.resize(
        info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> WearableSensorPackageHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // export sensor state interface
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
    }

    return state_interfaces;
}

hardware_interface::CallbackReturn WearableSensorPackageHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{

    // TODO: Open BLE connection to WSP

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WearableSensorPackageHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{

    // TODO: Close BLE connection

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WearableSensorPackageHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    // TODO: Read sensor data into hw_sensor_states_

    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    bearmax_hardware::WearableSensorPackageHardware,
    hardware_interface::SensorInterface)
