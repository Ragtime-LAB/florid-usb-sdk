#ifndef HYSERIAL_SOCKET_HPP
#define HYSERIAL_SOCKET_HPP

#include <tl/expected.hpp>
#include <HySerial/Util/Error.hpp>
#include <HySerial/Interface/Config.hpp>

namespace HySerial
{
    class Socket
    {
    public:
        explicit Socket(const SerialConfig& config);
        Socket() = delete;
        ~Socket();
        tl::expected<void, Error> ensure_connected() noexcept;
        tl::expected<void, Error> validate_connection() noexcept;
        [[nodiscard]] tl::expected<void, Error> flush() const noexcept;
        int sock_fd{};

    private:
        SerialConfig config;
    };
}

#endif //HYSERIAL_SOCKET_HPP
