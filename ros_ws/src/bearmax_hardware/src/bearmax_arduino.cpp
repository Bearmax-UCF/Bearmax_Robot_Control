#include "bearmax_hardware/bearmax_arduino.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bearmax_hardware
{
    // Used for array indexes! Don't change numbers!
    enum Joint {
        CHASSIS = 0,
        HEAD_ROLL,
        HEAD_PITCH,
        HEAD_YAW,
        LEFT_EAR_YAW,
        LEFT_EAR_PITCH,
        RIGHT_EAR_YAW,
        RIGHT_EAR_PITCH,
        LEFT_ARM_SHOULDER,
        LEFT_ARM_ROTATOR,
        LEFT_ARM_ELBOW,
        RIGHT_ARM_SHOULDER,
        RIGHT_ARM_ROTATOR,
        RIGHT_ARM_ELBOW,
        NUMBER_OF_JOINTS /* not a valid index! */
    };

#define CMD_INTERFACE(X,Y) command_interfaces.emplace_back( hardware_interface::CommandInterface(X, hardware_interface::HW_IF_POSITION, &hw_servo_cmds_[Y]))
#define STATE_INTERFACE(X,Y) state_interfaces.emplace_back( hardware_interface::StateInterface(X, hardware_interface::HW_IF_POSITION, &hw_servo_states_[Y]))


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
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    hw_servo_cmds_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Setup Arduino
    arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

    if (arduino_.connected()) {
        RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Already connected!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Not yet connected!");
    }

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Finished Initialization");

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> BearmaxArduino::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

/*
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_servo_states_[i]
            )
        );
    }
    */

    STATE_INTERFACE("chassis_joint", Joint::CHASSIS);
    STATE_INTERFACE("head_platform_roll_joint", Joint::HEAD_ROLL);
    STATE_INTERFACE("head_platform_pitch_joint", Joint::HEAD_PITCH);
    STATE_INTERFACE("head_platform_yaw_joint", Joint::HEAD_YAW);
    STATE_INTERFACE("left_ear_rotator_joint", Joint::LEFT_EAR_YAW);
    STATE_INTERFACE("left_ear_pitch_joint", Joint::LEFT_EAR_PITCH);
    STATE_INTERFACE("right_ear_rotator_joint", Joint::RIGHT_EAR_YAW);
    STATE_INTERFACE("right_ear_pitch_joint", Joint::RIGHT_EAR_PITCH);
    STATE_INTERFACE("left_arm_shoulder_joint", Joint::LEFT_ARM_SHOULDER);
    STATE_INTERFACE("left_arm_rotator_joint", Joint::LEFT_ARM_ROTATOR);
    STATE_INTERFACE("left_arm_elbow_joint", Joint::LEFT_ARM_ELBOW);
    STATE_INTERFACE("right_arm_shoulder_joint", Joint::RIGHT_ARM_SHOULDER);
    STATE_INTERFACE("right_arm_rotator_joint", Joint::RIGHT_ARM_ROTATOR);
    STATE_INTERFACE("right_arm_elbow_joint", Joint::RIGHT_ARM_ELBOW);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BearmaxArduino::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    /*
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_servo_cmds_[i]
            )
        );
    }
    */

    CMD_INTERFACE("chassis_joint", Joint::CHASSIS);
    CMD_INTERFACE("head_platform_roll_joint", Joint::HEAD_ROLL);
    CMD_INTERFACE("head_platform_pitch_joint", Joint::HEAD_PITCH);
    CMD_INTERFACE("head_platform_yaw_joint", Joint::HEAD_YAW);
    CMD_INTERFACE("left_ear_rotator_joint", Joint::LEFT_EAR_YAW);
    CMD_INTERFACE("left_ear_pitch_joint", Joint::LEFT_EAR_PITCH);
    CMD_INTERFACE("right_ear_rotator_joint", Joint::RIGHT_EAR_YAW);
    CMD_INTERFACE("right_ear_pitch_joint", Joint::RIGHT_EAR_PITCH);
    CMD_INTERFACE("left_arm_shoulder_joint", Joint::LEFT_ARM_SHOULDER);
    CMD_INTERFACE("left_arm_rotator_joint", Joint::LEFT_ARM_ROTATOR);
    CMD_INTERFACE("left_arm_elbow_joint", Joint::LEFT_ARM_ELBOW);
    CMD_INTERFACE("right_arm_shoulder_joint", Joint::RIGHT_ARM_SHOULDER);
    CMD_INTERFACE("right_arm_rotator_joint", Joint::RIGHT_ARM_ROTATOR);
    CMD_INTERFACE("right_arm_elbow_joint", Joint::RIGHT_ARM_ELBOW);

    return command_interfaces;
}

hardware_interface::CallbackReturn BearmaxArduino::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!arduino_.connected()) {
        arduino_.connect();
    }

    // TODO: Maybe add an 'activate' command to arduino to power on the servos?

    // Set default values.
    for (auto i = 0u; i < hw_servo_states_.size(); i++) {
        if (std::isnan(hw_servo_states_[i])) {
            hw_servo_states_[i] = 0;
            hw_servo_cmds_[i] = 0;
        }
    }

    /*
    std::vector<uint8_t> res;

    std::stringstream ss;

    ss << res.size() << " ";

    size_t n = arduino_.testRead(res);

    ss << n << " " << res.size();

    std::string sService = ss.str();

    std::string res_str(res.begin(), res.end());

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Reading Servo Values!");
    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), sService.c_str());
    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), res_str.c_str());
    */

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearmaxArduino::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (arduino_.connected()) {
        arduino_.disconnect();
    }

    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Deactivated!");

    // TODO: Maybe add an 'deactivate' command to arduino to power off the servos?

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BearmaxArduino::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!arduino_.connected()) {
        return hardware_interface::return_type::ERROR;
    }

//    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Reading Servo Values!");

    //arduino_.getServoValues(hw_servo_states_);

    for (uint i = 0; i < hw_servo_states_.size(); i++) {
        hw_servo_states_[i] = hw_servo_cmds_[i];
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type BearmaxArduino::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!arduino_.connected()) {
        return hardware_interface::return_type::ERROR;
    }

//    RCLCPP_INFO(rclcpp::get_logger("BearmaxArduino"), "Writing Servo Values!");

    std::string res = arduino_.setServoValues(hw_servo_cmds_);

    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    bearmax_hardware::BearmaxArduino,
    hardware_interface::SystemInterface)

