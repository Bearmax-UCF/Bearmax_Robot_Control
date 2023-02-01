#ifndef BEARMAX_HARDWARE__ARDUINO_COMMS_HPP_
#define BEARMAX_HARDWARE__ARDUINO_COMMS_HPP_

#include <serial/serial.h>
#include <string>
#include <cstring>
#include <vector>

namespace bearmax_hardware
{
    class ArduinoComms
    {
        public:
            ArduinoComms();

            ArduinoComms(const std::string &serial_device, int baud_rate, int timeout_ms);

            void setup(const std::string &serial_device, int baud_rate, int timeout_ms);
            void sendEmptyMsg();
            void setServoValues(std::vector<int> v);

            bool connected() const { return serial_conn_.isOpen(); }

            std::string sendMsg(const std::string &msg);
        private:
            serial::Serial serial_conn_;
    };
}
#endif
