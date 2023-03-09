#ifndef BEARMAX_HARDWARE__ARDUINO_COMMS_HPP_
#define BEARMAX_HARDWARE__ARDUINO_COMMS_HPP_

#include <serial_driver/serial_driver.hpp>
#include <string>
#include <cstring>
#include <vector>

using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;

namespace bearmax_hardware
{
    class ArduinoComms
    {
        public:
            ArduinoComms();

            ~ArduinoComms();

            ArduinoComms(const std::string &serial_device, int baud_rate, int timeout_ms);

            void setup(const std::string &serial_device, int baud_rate, int timeout_ms);
            void sendEmptyMsg();
            void setServoValues(std::vector<double> v);
            void getServoValues(std::vector<double>& v);

            bool connected() const { return serial_driver_->port()->is_open(); }
            void connect() const { serial_driver_->port()->open(); }
            void disconnect() const { serial_driver_->port()->close(); }

            std::string sendMsg(const std::string &msg);
        private:
            std::unique_ptr<IoContext> owned_ctx_{};
            std::unique_ptr<SerialDriver> serial_driver_;
            std::unique_ptr<SerialPortConfig> serial_config_;
    };
}
#endif
