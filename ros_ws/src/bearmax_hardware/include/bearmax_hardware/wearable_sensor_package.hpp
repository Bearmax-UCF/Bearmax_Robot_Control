#ifndef BEARMAX_HARDWARE__WEARABLE_SENSOR_PACKAGE_HPP_
#define BEARMAX_HARDWARE__WEARABLE_SENSOR_PACKAGE_HPP_

#include <memory>
#include <string>
#include <vector>

// TODO: Include libraries for BLE communication

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "bearmax_hardware/visibility_control.h"

namespace bearmax_hardware
{
  class WearableSensorPackageHardware : public hardware_interface::SensorInterface
  {
    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(WearableSensorPackageHardware)

      BEARMAX_HARDWARE_PUBLIC
       hardware_interface::CallbackReturn on_init(
           const hardware_interface::HardwareInfo & info) override;

      BEARMAX_HARDWARE_PUBLIC
       std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      BEARMAX_HARDWARE_PUBLIC
       hardware_interface::CallbackReturn on_activate(
           const rclcpp_lifecycle::State & previous_state) override;

      BEARMAX_HARDWARE_PUBLIC
       hardware_interface::CallbackReturn on_deactivate(
           const rclcpp_lifecycle::State & previous_state) override;

      BEARMAX_HARDWARE_PUBLIC
       hardware_interface::return_type read(
           const rclcpp::Time & time, const rclcpp::Duration & period) override;
    private:
      std::string hw_device_id_;
      std::string hw_gsr_uuid_;
      // dummy variable
      std::vector<double> hw_sensor_states_;
      double gsr = 0;
  };
}

#endif
