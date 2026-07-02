#include <HySerial/Interface/Interface.hpp>

namespace HySerial
{
    tl::expected<std::unique_ptr<Serial>, Error> Serial::create(SerialConfig cfg,
                                                                std::unique_ptr<Socket> socket,
                                                                std::unique_ptr<UringManager> manager)
    {
        if (!socket)
        {
            return tl::make_unexpected(Error{ErrorCode::SocketCreateError, "Null socket"});
        }
        if (!manager)
        {
            return tl::make_unexpected(Error{ErrorCode::UringInitError, "Null uring manager"});
        }

        // Before creating Serial, bind the socket fd to the manager so submit_send can work
        if (socket->sock_fd > 0)
        {
            manager->bind_fd(socket->sock_fd);
        }

        // Create Serial instance (do not start automatic read here)
        auto serial = std::unique_ptr<Serial>(new Serial(std::move(cfg), std::move(socket), std::move(manager)));

        // Start uring manager loop in a jthread
        serial->m_thread = std::jthread([mgr = serial->m_uring.get()](std::stop_token)
        {
            mgr->run();
        });

        return serial;
    }

    Serial::Serial(SerialConfig cfg, std::unique_ptr<Socket> socket, std::unique_ptr<UringManager> manager)
        : m_cfg(std::move(cfg)), m_socket(std::move(socket)), m_uring(std::move(manager))
    {
    }

    Serial::~Serial()
    {
        if (m_uring)
        {
            m_uring->stop();
        }
        // Ensure the background thread has stopped BEFORE destroying the UringManager
        if (m_thread.joinable())
        {
            m_thread.join();
        }
        // jthread destructor would also join on its own, but we must join here to avoid
        // the background thread accessing m_uring after we've destroyed it.
        m_uring.reset();
        m_socket.reset();
    }

    void Serial::send(std::span<const std::byte> data)
    {
            m_uring->submit_send(data);
    }

    void Serial::start_read(size_t buf_size)
    {
        if (m_socket->sock_fd > 0)
        {
            m_uring->start_read_for_fd(m_socket->sock_fd, buf_size);
        }
    }

    void Serial::stop_read()
    {
        if (m_uring)
            m_uring->stop_read_for_fd();
    }

    void Serial::set_read_callback(ReadCallback cb)
    {
        if (m_uring)
            m_uring->register_read_callback(std::move(cb));
    }

    void Serial::set_send_callback(WriteCallback cb)
    {
        if (m_uring)
            m_uring->register_write_callback(std::move(cb));
    }

    void Serial::set_error_callback(ErrorCallback cb)
    {
        if (m_uring)
            m_uring->register_error_callback(std::move(cb));
    }
}
