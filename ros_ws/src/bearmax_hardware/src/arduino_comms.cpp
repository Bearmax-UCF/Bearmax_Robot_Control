#include "bearmax_hardware/arduino_comms.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

namespace bearmax_hardware
{
    void ArduinoComms::setup(const std::string &serial_device, int baud_rate, int timeout_ms)
    {
        serial_conn_.setPort(serial_device);
        serial_conn_.setBaudrate(baud_rate);
        serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
        serial_conn_.setTimeout(tt);
        serial_conn_.open();
    }

    void ArduinoComms::sendEmptyMsg()
    {
        std::string res = sendMsg("\n");
    }

    void ArduinoComms::setServoValues(std::vector<int> v)
    {
        std::stringstream ss;
        ss << "s ";

        for (uint i = 0; i < v.size(); i++) {
            if (i > 0) {
                ss << ":";
            }
            ss << v[i];
        }

        ss << "\n";

        sendMsg(ss.str());
    }

    std::string ArduinoComms::sendMsg(const std::string &msg)
    {
        serial_conn_.write(msg);
        std::string res = serial_conn_.readline();

        return res;
    }
}
