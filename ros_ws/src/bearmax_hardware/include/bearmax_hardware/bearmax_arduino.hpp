#ifndef BEARMAX_HARDWARE__BEARMAX_ARDUINO_HPP_
#define BEARMAX_HARDWARE__BEARMAX_ARDUINO_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "visibility_control.h"
#include "arduino_comms.hpp"

struct Config {
    int baud_rate = 57600;
    int timeout = 1000;
    std::string device = "/dev/ttyUSB0";
};

namespace bearmax_hardware
{
  class BearmaxArduino : public hardware_interface::SystemInterface
  {
    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(BearmaxArduino)

      BEARMAX_HARDWARE_PUBLIC
       hardware_interface::CallbackReturn on_init(
           const hardware_interface::HardwareInfo & info) override;

      BEARMAX_HARDWARE_PUBLIC
       std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      BEARMAX_HARDWARE_PUBLIC
       std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      BEARMAX_HARDWARE_PUBLIC
       hardware_interface::CallbackReturn on_activate(
           const rclcpp_lifecycle::State & previous_state) override;

      BEARMAX_HARDWARE_PUBLIC
       hardware_interface::CallbackReturn on_deactivate(
           const rclcpp_lifecycle::State & previous_state) override;

      BEARMAX_HARDWARE_PUBLIC
       hardware_interface::return_type read(
           const rclcpp::Time & time, const rclcpp::Duration & period) override;

      BEARMAX_HARDWARE_PUBLIC
       hardware_interface::return_type write(
           const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
      Config cfg_;
      ArduinoComms arduino_;

      std::vector<double> hw_servo_states_;
      std::vector<double> hw_servo_cmds_;
  };
}

#endif
