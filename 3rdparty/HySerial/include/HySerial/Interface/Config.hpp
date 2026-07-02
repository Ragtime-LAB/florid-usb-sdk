#ifndef HYSERIAL_CONFIG_HPP
#define HYSERIAL_CONFIG_HPP
#include <cstdint>
#include <string>

namespace HySerial
{
    enum class DataBits : uint8_t
    {
        BITS_5 = 5,
        BITS_6 = 6,
        BITS_7 = 7,
        BITS_8 = 8,
    };

    // 停止位枚举
    enum class StopBits : uint8_t
    {
        ONE = 1,
        TWO = 2,
    };

    // 校验位枚举
    enum class Parity : uint8_t
    {
        NONE = 0, // 无校验
        ODD = 1, // 奇校验
        EVEN = 2, // 偶校验
    };

    // 流控制枚举
    enum class FlowControl : uint8_t
    {
        NONE = 0, // 无
        RTS_CTS = 1, // RTS/CTS硬件流控
        XON_XOFF = 2, // XON/XOFF软件流控
    };

    struct SerialConfig
    {
        std::string device_path = "/dev/ttyUSB0";
        uint32_t baud_rate = 115200;
        DataBits data_bits = DataBits::BITS_8;
        StopBits stop_bits = StopBits::ONE;
        Parity parity = Parity::NONE;
        FlowControl flow_control = FlowControl::NONE;
        bool rts_dtr_on = false;
    };
}

#endif //HYSERIAL_CONFIG_HPP
