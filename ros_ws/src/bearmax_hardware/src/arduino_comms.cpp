#include "bearmax_hardware/arduino_comms.hpp"

#include <sstream>
#include <cstdlib>

namespace bearmax_hardware
{

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
        //serial_conn_.write(msg);
        //std::string res = serial_conn_.readline();
        std::vector<uint8_t> tx_buff(msg.begin(), msg.end());

 //       std::vector<uint8_t> rx_buff;

        serial_driver_->port()->send(tx_buff);

//        serial_driver_->port()->receive(std


        return "";
    }
}
