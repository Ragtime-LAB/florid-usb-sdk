#ifndef HYSERIAL_INTERFACE_HPP
#define HYSERIAL_INTERFACE_HPP

#include <memory>
#include <thread>
#include <tl/expected.hpp>

#include <HySerial/Interface/Config.hpp>
#include <HySerial/Socket/Socket.hpp>
#include <HySerial/Interface/UringManager.hpp>
#include <HySerial/Util/Error.hpp>

namespace HySerial
{
    class Builder;

    class Serial
    {
    public:
        // Factory that creates Serial and starts the UringManager thread
        static tl::expected<std::unique_ptr<Serial>, Error> create(SerialConfig cfg,
                                                                   std::unique_ptr<Socket> socket,
                                                                   std::unique_ptr<UringManager> manager);

        // Non-copyable
        Serial(const Serial&) = delete;
        Serial& operator=(const Serial&) = delete;

        // movable
        Serial(Serial&&) = default;
        Serial& operator=(Serial&&) = default;

        ~Serial();

        // Send data (thread-safe wrt UringManager usage)
        void send(std::span<const std::byte> data);

        // Explicit control for continuous read
        // start_read will bind the manager to the socket fd and begin continuous reading
        void start_read(size_t buf_size = 4096);
        // stop automatic continuous reading
        void stop_read();

        // Set callbacks after creation (thread-safe via UringManager's registration)
        void set_read_callback(ReadCallback cb);
        void set_send_callback(WriteCallback cb);
        void set_error_callback(ErrorCallback cb);

    private:
        friend class Builder;
        explicit Serial(SerialConfig cfg, std::unique_ptr<Socket> socket, std::unique_ptr<UringManager> manager);

        SerialConfig m_cfg;
        std::unique_ptr<Socket> m_socket;
        std::unique_ptr<UringManager> m_uring;
        std::jthread m_thread;
    };
}

#endif //HYSERIAL_INTERFACE_HPP
