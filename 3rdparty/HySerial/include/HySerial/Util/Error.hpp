#ifndef HYSERIAL_ERROR_HPP
#define HYSERIAL_ERROR_HPP

#include <string>

namespace HySerial
{
    enum class ErrorCode
    {
        SocketCreateError,
        InterfaceIndexError,
        SocketBindError,
        InvalidSocketError,
        SocketFlushError,

        UringInitError,
    };

    struct Error
    {
        Error(const ErrorCode c, std::string& msg) : code(c), message(std::move(msg))
        {
        }

        Error(const ErrorCode c, std::string&& msg) : code(c), message(std::move(msg))
        {
        }

        ErrorCode code;
        std::string message;
    };
}

#endif //HYSERIAL_ERROR_HPP
