#include "bearmax_hardware/arduino_comms.hpp"

#include <sstream>
#include <cstdlib>

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

    ArduinoComms::ArduinoComms()
        : owned_ctx_{new IoContext(2)}
        , serial_driver_{new SerialDriver(*owned_ctx_)}
    { }


    ArduinoComms::~ArduinoComms() {
        if (owned_ctx_) {
            owned_ctx_->waitForExit();
        }
    }

    void ArduinoComms::setup(const std::string &serial_device, int baud_rate, int /*timeout_ms*/)
    {

        serial_config_ = std::make_unique<SerialPortConfig>(
            baud_rate,
            drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE,
            drivers::serial_driver::StopBits::ONE
        );

        serial_driver_->init_port(
            serial_device,
            *serial_config_
            );

    }

    void ArduinoComms::sendEmptyMsg()
    {
        std::string res = sendMsg("\n");
    }

    void ArduinoComms::getServoValues(std::vector<double>& v)
    {

        std::string res = sendMsg("r\n");
        std::string delim = ":";

        size_t delim_idx = 0;

        for (size_t i = 0; i < Joint::NUMBER_OF_JOINTS; i++) {
            size_t next_delim_idx = 0;
            if (i == 0) {
                next_delim_idx = res.find(delim, 0);
            } else {
                next_delim_idx = res.find(delim, delim_idx + 1);
            }

            std::string token = "";
            if (i == 0) {
                token = res.substr(0, next_delim_idx);
            } else {
                token = res.substr(delim_idx + 1, next_delim_idx - delim_idx);
            }

            v[i] = (double) std::atoi(token.c_str());

            delim_idx = next_delim_idx;
        }

    }

    void ArduinoComms::setServoValues(std::vector<double> v)
    {
        std::stringstream ss;
        ss << "s ";

        for (uint i = 0; i < v.size(); i++) {
            if (i > 0) {
                ss << ":";
            }
            ss << (int)v[i];
        }

        ss << "\n";

        sendMsg(ss.str());
    }

    std::string ArduinoComms::sendMsg(const std::string &msg)
    {
        std::vector<uint8_t> tx_buff(msg.begin(), msg.end());

        std::vector<uint8_t> rx_buff;

        serial_driver_->port()->send(tx_buff);

        serial_driver_->port()->receive(rx_buff);

        std::string rx_str(rx_buff.begin(), rx_buff.end());

        return rx_str;
    }
}
